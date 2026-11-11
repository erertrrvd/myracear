r"""笔记本摄像头：YOLO11n 三类模型（light_box 灯箱 / red_on 亮红灯 / green_on 亮绿灯）。
判断逻辑：
  1. 灯箱框先过几何过滤（长宽比、高度），通过的才算灯箱
  2. 亮灯框的中心必须落在某个通过过滤的灯箱框里，才算“该灯亮”
  3. 灯箱里有亮红灯 -> red；亮绿灯 -> green；红绿同时 -> conflict（不放行）
     有灯箱但没亮灯 -> off；没有灯箱 -> none
  4. 时间滤波：连续 N 帧一致才切换显示的状态
  5. 放行规则：只有状态是 green 才显示 GO，其余一律 STOP
运行：python 脚本\light_yolo11_cam.py
按 q 退出。
"""
import argparse
import os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # 视觉模型 文件夹
import cv2
from ultralytics import YOLO

ap = argparse.ArgumentParser()
ap.add_argument("--model", default=os.path.join(ROOT, "模型", "yolo11n_best.pt"))
ap.add_argument("--conf-box", type=float, default=0.5, help="灯箱置信度阈值")
ap.add_argument("--conf-lamp", type=float, default=0.5, help="亮灯（红/绿）置信度阈值")
ap.add_argument("--cam", type=int, default=0)
ap.add_argument("--ar-min", type=float, default=2.5)
ap.add_argument("--ar-max", type=float, default=4.5)
ap.add_argument("--cy-min", type=float, default=0.20)
ap.add_argument("--cy-max", type=float, default=0.85)
ap.add_argument("--confirm", type=int, default=3, help="连续几帧一致才切换状态")
ap.add_argument("--no-filter", action="store_true", help="关闭几何过滤")
a = ap.parse_args()

model = YOLO(a.model)
BOX, RED, GREEN = 0, 1, 2
COL = {RED: (0, 0, 255), GREEN: (0, 200, 0)}
SHOW = {"red": (0, 0, 255), "green": (0, 200, 0), "off": (200, 200, 200),
        "none": (120, 120, 120), "conflict": (0, 165, 255)}


def geom_ok(x1, y1, x2, y2, H):
    w, h = x2 - x1, y2 - y1
    if h <= 0:
        return False, "bad"
    ar = w / h
    cy = (y1 + y2) / 2 / H
    if not (a.ar_min <= ar <= a.ar_max):
        return False, f"ar{ar:.1f}"
    if not (a.cy_min <= cy <= a.cy_max):
        return False, f"cy{cy:.2f}"
    return True, ""


cap = cv2.VideoCapture(a.cam, cv2.CAP_DSHOW)
if not cap.isOpened():
    raise SystemExit("打不开摄像头")

shown, cand, cand_n = "none", "none", 0

while True:
    ok, frame = cap.read()
    if not ok:
        break
    H, W = frame.shape[:2]
    r = model.predict(frame, conf=0.05, imgsz=640, verbose=False)[0]

    boxes, lamps, rejected = [], [], 0
    for b in r.boxes:
        c = float(b.conf)
        cls = int(b.cls)
        x1, y1, x2, y2 = map(int, b.xyxy[0])
        thr = a.conf_box if cls == BOX else a.conf_lamp
        if c < thr:   # 低于阈值：暗灰细框，只用来看模型“差一点”认出了什么
            cv2.rectangle(frame, (x1, y1), (x2, y2), (90, 90, 90), 1)
            cv2.putText(frame, f"low {c:.2f}", (x1, min(y2 + 14, H - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (90, 90, 90), 1)
            continue
        if cls == BOX:
            ok_g, why = (True, "") if a.no_filter else geom_ok(x1, y1, x2, y2, H)
            if ok_g:
                boxes.append((x1, y1, x2, y2, c))
            else:
                rejected += 1
                cv2.rectangle(frame, (x1, y1), (x2, y2), (150, 150, 150), 1)
                cv2.putText(frame, f"x box {c:.2f} {why}", (x1, max(y1 - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
        else:
            lamps.append((cls, x1, y1, x2, y2, c))

    def in_box(l):
        cx, cy = (l[1] + l[3]) / 2, (l[2] + l[4]) / 2
        return any(bx1 <= cx <= bx2 and by1 <= cy <= by2 for (bx1, by1, bx2, by2, _) in boxes)

    for (bx1, by1, bx2, by2, bc) in boxes:
        cv2.rectangle(frame, (bx1, by1), (bx2, by2), (0, 255, 0), 1)
        cv2.putText(frame, f"box {bc:.2f}", (bx1, max(by1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    got = set()
    for l in lamps:
        cls, x1, y1, x2, y2, c = l
        if in_box(l):
            got.add(cls)
            cv2.rectangle(frame, (x1, y1), (x2, y2), COL[cls], 2)
            cv2.putText(frame, f"{'red' if cls == RED else 'green'} {c:.2f}", (x1, min(y2 + 16, H - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, COL[cls], 2)
        else:   # 不在灯箱里的亮灯框：灰色，说明被忽略
            cv2.rectangle(frame, (x1, y1), (x2, y2), (150, 150, 150), 1)
            cv2.putText(frame, f"x lamp {c:.2f} (not in box)", (x1, max(y1 - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)

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

    if state == cand:
        cand_n += 1
    else:
        cand, cand_n = state, 1
    if cand_n >= a.confirm:
        shown = cand

    cv2.putText(frame, f"STATE: {shown}   (raw: {state}, rejected boxes: {rejected})", (10, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, SHOW[shown], 2)
    go = shown == "green"
    cv2.putText(frame, "GO" if go else "STOP", (10, 62),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 0) if go else (0, 0, 255), 3)
    cv2.imshow("YOLO11 light test (q to quit)", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
