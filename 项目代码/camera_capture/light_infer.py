#!/usr/bin/env python3
"""小车上的红绿灯识别：YOLOv5n（3 类）+ 状态判断。

模型类别（顺序必须和训练一致）：0 light_box 灯箱 / 1 red_on 亮红灯 / 2 green_on 亮绿灯

状态判断：
  1. 灯箱框先过几何过滤（长宽比、中心高度），通过的才算灯箱
  2. 亮灯框中心必须落在某个通过过滤的灯箱框里，才算“该灯亮”
  3. 亮红灯 -> red；亮绿灯 -> green；红绿同时 -> conflict；有灯箱没亮灯 -> off；没灯箱 -> none
  4. 连续 N 帧一致才切换（--confirm）
  5. 只有 green 才放行（GO），其余一律 STOP

用法（在小车上，root 的 conda python3；先 source CANN 环境，并关掉其他占用摄像头的程序）：
  source /usr/local/Ascend/ascend-toolkit/set_env.sh
  python3 light_infer.py --model /home/racecar/light_infer/output/yolov5n_bs1.om            # 摄像头，终端看状态
  python3 light_infer.py --model xxx.om --show                                              # 弹窗（需要 MobaXterm X server，较慢）
  python3 light_infer.py --model xxx.om --save-video /tmp/out.mp4 --max-seconds 30          # 存带框视频，拷回电脑看
  python3 light_infer.py --model xxx.om --source /path/to/images                            # 离线跑一个文件夹的图片
  python3 light_infer.py --model best.onnx --source /path/to/images                         # 电脑上用 onnxruntime 验证 ONNX

每帧把结果写到 --state-file（默认 /tmp/light_state.txt，先写临时文件再改名），给第二阶段的 ROS 节点读。
"""
import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from det_utils import letterbox, non_max_suppression, scale_coords  # noqa: E402  课程自带，Apache-2.0

NAMES = ["light_box", "red_on", "green_on"]
BOX, RED, GREEN = 0, 1, 2
INPUT_SHAPE = (640, 640)


# ---------------------------------------------------------------- 模型后端
class Backend:
    """.om -> ais_bench（昇腾）；.onnx -> onnxruntime（电脑上验证用）"""

    def __init__(self, path, device_id=0):
        self.is_om = path.endswith(".om")
        if self.is_om:
            from ais_bench.infer.interface import InferSession
            self.sess = InferSession(device_id, path)
        else:
            import onnxruntime as ort
            self.sess = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
            self.in_name = self.sess.get_inputs()[0].name

    def __call__(self, batch_chw):
        """batch_chw: float32 [1,3,640,640]，0~1；返回 torch.Tensor [1,N,5+nc]"""
        if self.is_om:
            out = self.sess.infer([batch_chw.astype(np.float16)])
        else:
            out = self.sess.run(None, {self.in_name: batch_chw.astype(np.float32)})
        return torch.tensor(np.asarray(out[0], dtype=np.float32))


def preprocess(frame):
    img, ratio, pad = letterbox(frame, new_shape=INPUT_SHAPE)
    x = img[..., ::-1].transpose(2, 0, 1) / 255.0           # BGR->RGB, HWC->CHW
    return np.ascontiguousarray(x[None]).astype(np.float32), ratio, pad


def detect(backend, frame, low_conf=0.25, iou=0.5):
    """返回 [(cls, conf, x1, y1, x2, y2)]，坐标是原图像素；按类别阈值的筛选在 decide() 里做"""
    x, ratio, pad = preprocess(frame)
    out = backend(x)
    boxes = non_max_suppression(out, conf_thres=low_conf, iou_thres=iou, multi_label=False)[0].numpy()
    if len(boxes) == 0:
        return []
    scale_coords(INPUT_SHAPE, boxes[:, :4], frame.shape, ratio_pad=(ratio, pad))
    return [(int(b[5]), float(b[4]), float(b[0]), float(b[1]), float(b[2]), float(b[3])) for b in boxes]


# ---------------------------------------------------------------- 状态判断（纯函数，不依赖模型）
def decide(dets, W, H, a):
    """返回 (state, box, info)。box = 通过过滤、置信度最高的灯箱 (x1,y1,x2,y2,conf) 或 None"""
    boxes, lamps, rejected = [], [], []
    for cls, c, x1, y1, x2, y2 in dets:
        if cls == BOX:
            if c < a.conf_box:
                continue
            w, h = x2 - x1, y2 - y1
            ar = w / h if h > 0 else 0
            cy = (y1 + y2) / 2 / H
            if a.no_filter or (a.ar_min <= ar <= a.ar_max and a.cy_min <= cy <= a.cy_max):
                boxes.append((x1, y1, x2, y2, c))
            else:
                rejected.append((x1, y1, x2, y2, c, f"ar{ar:.1f}" if not (a.ar_min <= ar <= a.ar_max) else f"cy{cy:.2f}"))
        elif cls in (RED, GREEN) and c >= a.conf_lamp:
            lamps.append((cls, c, x1, y1, x2, y2))

    got, used = set(), []
    for cls, c, x1, y1, x2, y2 in lamps:
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        if any(bx1 <= cx <= bx2 and by1 <= cy <= by2 for bx1, by1, bx2, by2, _ in boxes):
            got.add(cls)
            used.append((cls, c, x1, y1, x2, y2))

    if not boxes:
        state = "none"
    elif got == {RED, GREEN}:
        state = "conflict"
    elif RED in got:
        state = "red"
    elif GREEN in got:
        state = "green"
    else:
        state = "off"
    best = max(boxes, key=lambda b: b[4]) if boxes else None
    return state, best, dict(lamps=used, rejected=rejected, boxes=boxes)


class Smoother:
    def __init__(self, n):
        self.n, self.shown, self.cand, self.cnt = n, "none", "none", 0

    def update(self, s):
        if s == self.cand:
            self.cnt += 1
        else:
            self.cand, self.cnt = s, 1
        if self.cnt >= self.n:
            self.shown = self.cand
        return self.shown


def write_state(path, ts, state, raw, box, lamp_conf, fps, W, H):
    """原子写：timestamp state raw x1 y1 x2 y2 box_conf lamp_conf fps W H（没灯箱时框为 -1）"""
    if box:
        bx = f"{box[0]:.0f} {box[1]:.0f} {box[2]:.0f} {box[3]:.0f} {box[4]:.3f}"
    else:
        bx = "-1 -1 -1 -1 0"
    line = f"{ts:.3f} {state} {raw} {bx} {lamp_conf:.3f} {fps:.1f} {W} {H}\n"
    tmp = path + ".tmp"
    try:
        with open(tmp, "w") as f:
            f.write(line)
        os.replace(tmp, path)
    except OSError as e:           # 写文件失败不能让识别停下来
        print("写状态文件失败：", e)


def draw(frame, state, shown, box, info, fps, infer_ms):
    for x1, y1, x2, y2, c, why in info["rejected"]:
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (150, 150, 150), 1)
        cv2.putText(frame, f"x box {c:.2f} {why}", (int(x1), max(int(y1) - 6, 12)), 0, 0.45, (150, 150, 150), 1)
    for x1, y1, x2, y2, c in info["boxes"]:
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 1)
    for cls, c, x1, y1, x2, y2 in info["lamps"]:
        col = (0, 0, 255) if cls == RED else (0, 200, 0)
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), col, 2)
    go = shown == "green"
    cv2.putText(frame, f"STATE: {shown} (raw {state})  {fps:.1f}fps {infer_ms:.0f}ms", (8, 24), 0, 0.65, (0, 255, 255), 2)
    cv2.putText(frame, "GO" if go else "STOP", (8, 58), 0, 1.0, (0, 200, 0) if go else (0, 0, 255), 3)


# ---------------------------------------------------------------- 取图
def frames(source, cam_w, cam_h):
    """产生 (名字, 图像)；source 是摄像头编号 / 视频文件 / 图片文件夹"""
    if os.path.isdir(source):
        for p in sorted(glob.glob(os.path.join(source, "*.jpg")) + glob.glob(os.path.join(source, "*.png"))):
            img = cv2.imdecode(np.fromfile(p, np.uint8), cv2.IMREAD_COLOR)   # imread 读不了含中文的路径
            if img is not None:
                yield os.path.basename(p), img
        return
    cap = cv2.VideoCapture(int(source) if source.isdigit() else source)
    if not cap.isOpened():
        raise SystemExit(f"打不开 {source}（摄像头可能被别的程序占用）")
    if source.isdigit():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_h)
    n = 0
    while True:
        ok, f = cap.read()
        if not ok:
            break
        yield str(n), f
        n += 1
    cap.release()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", required=True, help=".om（车上）或 .onnx（电脑验证）")
    ap.add_argument("--source", default="0", help="摄像头编号 / 视频文件 / 图片文件夹")
    ap.add_argument("--device-id", type=int, default=0)
    ap.add_argument("--conf-box", type=float, default=0.5)
    ap.add_argument("--conf-lamp", type=float, default=0.5)
    ap.add_argument("--confirm", type=int, default=3, help="连续几帧一致才切换状态（图片文件夹模式固定为 1）")
    ap.add_argument("--ar-min", type=float, default=2.5)
    ap.add_argument("--ar-max", type=float, default=4.5)
    ap.add_argument("--cy-min", type=float, default=0.20)
    ap.add_argument("--cy-max", type=float, default=0.85)
    ap.add_argument("--no-filter", action="store_true")
    ap.add_argument("--state-file", default="/tmp/light_state.txt")
    ap.add_argument("--show", action="store_true", help="弹窗显示（需要 X server，很慢）")
    ap.add_argument("--save-video", default="", help="把带框画面存成 mp4")
    ap.add_argument("--max-seconds", type=float, default=0, help="0 = 一直跑")
    ap.add_argument("--cam-width", type=int, default=640)
    ap.add_argument("--cam-height", type=int, default=480)
    a = ap.parse_args()

    backend = Backend(a.model, a.device_id)
    is_dir = os.path.isdir(a.source)
    sm = Smoother(1 if is_dir else a.confirm)
    writer, show = None, a.show
    last_state, t0, t_print, nfr, infer_acc = None, time.time(), time.time(), 0, 0.0
    print(f"模型 {a.model}  来源 {a.source}  Ctrl+C 结束")

    try:
        for name, frame in frames(a.source, a.cam_width, a.cam_height):
            H, W = frame.shape[:2]
            t1 = time.time()
            dets = detect(backend, frame)
            infer_ms = (time.time() - t1) * 1000
            raw, box, info = decide(dets, W, H, a)
            shown = sm.update(raw)
            nfr += 1
            infer_acc += infer_ms
            now = time.time()
            fps = nfr / max(now - t0, 1e-6)
            lamp_conf = max([l[1] for l in info["lamps"]], default=0.0)
            write_state(a.state_file, now, shown, raw, box, lamp_conf, fps, W, H)

            if is_dir:
                print(f"{name}: {raw}")
            elif shown != last_state:
                print(f"[{time.strftime('%H:%M:%S')}] 状态 {last_state} -> {shown}   {'GO' if shown == 'green' else 'STOP'}")
            last_state = shown
            if not is_dir and now - t_print >= 5:
                print(f"  {fps:.1f} fps，平均推理 {infer_acc / nfr:.0f} ms，状态 {shown}")
                t_print = now

            if show or a.save_video:
                draw(frame, raw, shown, box, info, fps, infer_ms)
            if a.save_video:
                if writer is None:
                    writer = cv2.VideoWriter(a.save_video, cv2.VideoWriter_fourcc(*"mp4v"), 15, (W, H))
                writer.write(frame)
            if show:
                try:
                    cv2.imshow("light (q to quit)", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                except cv2.error as e:
                    print("无法弹窗（没有 X server？），关闭 --show：", str(e)[:80])
                    show = False
            if a.max_seconds and now - t0 >= a.max_seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows() if show else None
    if nfr:
        print(f"结束：{nfr} 帧，平均 {nfr / (time.time() - t0):.1f} fps，平均推理 {infer_acc / nfr:.0f} ms")


if __name__ == "__main__":
    main()
