#!/usr/bin/env python3
"""电脑端：连小车收摄像头画面 → YOLOv5n 识别 → 把结果传回小车。

判断逻辑（几何过滤、亮灯框必须在灯箱框内、连续 N 帧才切换）直接复用 light_infer.py，不重复写。
电脑只作客户端主动连小车（出站连接，不会被 Windows 防火墙拦）；断线自动重连。
画面和框在电脑本机窗口显示，没有 X11 的卡顿。

运行（电脑 PowerShell，用装了 onnxruntime 和 torch 的 venv）：
  D:/light_ai/venv/Scripts/python.exe pc_vision_client.py --car 192.168.5.100
  加 --no-show 不弹窗；--port 9000 对应小车端的端口；q 键退出窗口。
"""
import argparse
import os
import socket
import sys
import threading
import time
import types

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import remote_proto as rp
import light_infer as li

DEFAULT_MODEL = "D:/VSCode/ros2技术集/智能车总文件/视觉模型/模型/yolov5n_best.onnx"


class LatestFrame:
    """接收线程只保留最新一帧；识别慢时中间帧直接跳过，不排队，延迟不会越积越大"""

    def __init__(self):
        self.cv = threading.Condition()
        self.item, self.n_recv, self.closed = None, 0, False

    def put(self, fid, jpeg):
        with self.cv:
            self.item = (fid, jpeg)
            self.n_recv += 1
            self.cv.notify_all()

    def close(self):
        with self.cv:
            self.closed = True
            self.cv.notify_all()

    def take(self, last, timeout=0.5):
        with self.cv:
            self.cv.wait_for(lambda: self.closed or (self.item is not None and self.item[0] != last), timeout)
            if self.item is not None and self.item[0] != last:
                return self.item
            return None


def reader(sock, frames):
    try:
        while True:
            fid, _t, jpeg = rp.recv_frame(sock)
            frames.put(fid, jpeg)
    except (OSError, ValueError):
        pass
    finally:
        frames.close()


def make_args(ns):
    """给 light_infer.decide 用的参数对象"""
    return types.SimpleNamespace(conf_box=ns.conf_box, conf_lamp=ns.conf_lamp, ar_min=ns.ar_min, ar_max=ns.ar_max,
                                 cy_min=ns.cy_min, cy_max=ns.cy_max, no_filter=ns.no_filter)


def session(ns, backend, decide_args):
    sock = socket.create_connection((ns.car, ns.port), timeout=3)
    sock.settimeout(None)
    rp.enable_keepalive(sock)
    print(f"已连接小车 {ns.car}:{ns.port}")
    frames = LatestFrame()
    threading.Thread(target=reader, args=(sock, frames), daemon=True).start()
    sm, last_id, last_state, fps, t_prev = li.Smoother(ns.confirm), 0, None, 0.0, time.time()
    n_proc, n_rep, t_rep = 0, 0, time.time()
    while True:
        item = frames.take(last_id)
        if item is None:
            if frames.closed:
                raise ConnectionError("小车已断开")
            continue
        fid, jpeg = item
        last_id = fid
        frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        H, W = frame.shape[:2]
        t0 = time.time()
        dets = li.detect(backend, frame)
        raw, box, info = li.decide(dets, W, H, decide_args)
        shown = sm.update(raw)
        pc_ms = (time.time() - t0) * 1000
        rp.send_result(sock, {"id": fid, "state": shown, "raw": raw,
                              "box": [float(v) for v in box] if box else None,
                              "lamp_conf": max([l[1] for l in info["lamps"]], default=0.0),
                              "pc_ms": round(pc_ms, 1), "w": W, "h": H})
        n_proc += 1
        now = time.time()
        fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-6) if fps else 1.0 / max(now - t_prev, 1e-6)
        t_prev = now
        if shown != last_state:
            print(f"[{time.strftime('%H:%M:%S')}] 状态 {last_state} -> {shown}  {'GO' if shown == 'green' else 'STOP'}")
            last_state = shown
        if now - t_rep >= 5:
            skipped = (frames.n_recv - n_rep) - n_proc
            print(f"  识别 {n_proc / (now - t_rep):.1f} fps，单帧 {pc_ms:.0f} ms，跳过 {max(skipped, 0)} 帧，状态 {shown}")
            n_proc, n_rep, t_rep = 0, frames.n_recv, now
        if not ns.no_show:
            li.draw(frame, raw, shown, box, info, fps, pc_ms)
            cv2.imshow("pc vision (q to quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                sock.close()
                return True
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--car", default="192.168.5.100", help="小车 IP")
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--model", default=DEFAULT_MODEL, help="yolov5n 的 .onnx")
    ap.add_argument("--conf-box", type=float, default=0.5)
    ap.add_argument("--conf-lamp", type=float, default=0.5)
    ap.add_argument("--confirm", type=int, default=3, help="连续几帧一致才切换状态")
    ap.add_argument("--ar-min", type=float, default=2.5)
    ap.add_argument("--ar-max", type=float, default=4.5)
    ap.add_argument("--cy-min", type=float, default=0.20)
    ap.add_argument("--cy-max", type=float, default=0.85)
    ap.add_argument("--no-filter", action="store_true")
    ap.add_argument("--no-show", action="store_true")
    ns = ap.parse_args()

    backend, decide_args = li.Backend(ns.model), make_args(ns)
    print(f"模型 {ns.model}；连接 {ns.car}:{ns.port}，Ctrl+C 或窗口里按 q 退出")
    try:
        while True:
            try:
                if session(ns, backend, decide_args):
                    break
            except (OSError, ConnectionError, ValueError) as e:
                print(f"连接中断或失败：{e}；2 秒后重试")
                time.sleep(2)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
