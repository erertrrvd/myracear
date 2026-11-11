r"""笔记本摄像头试验：只框红绿灯外框（整个灯箱），不框单个灯。
模型：yolov8m.pt（COCO 通用，类别 9 = traffic light，框的是整个灯箱）
红/绿：在灯箱框里按颜色（HSV）判断亮的是红灯还是绿灯。
运行：python 脚本\light_housing.py
按 q 退出。--conf 调阈值，默认 0.4。
"""
import argparse
import os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # 视觉模型 文件夹
import cv2
from ultralytics import YOLO

TL = 9  # COCO: traffic light

ap = argparse.ArgumentParser()
ap.add_argument("--model", default=os.path.join(ROOT, "模型", "通用模型试验", "yolov8m.pt"))
ap.add_argument("--conf", type=float, default=0.2)
ap.add_argument("--cam", type=int, default=0)
a = ap.parse_args()


def color_of(frame, x1, y1, x2, y2):
    crop = frame[max(y1, 0):y2, max(x1, 0):x2]
    if crop.size == 0:
        return "none", 0, 0
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    lit = (s > 80) & (v > 150)
    red = int((lit & ((h < 10) | (h > 170))).sum())
    green = int((lit & (h > 40) & (h < 90)).sum())
    if max(red, green) < 8:
        return "none", red, green
    return ("red" if red > green else "green"), red, green


model = YOLO(a.model)
cap = cv2.VideoCapture(a.cam, cv2.CAP_DSHOW)
if not cap.isOpened():
    raise SystemExit("打不开摄像头")

while True:
    ok, frame = cap.read()
    if not ok:
        break
    r = model.predict(frame, conf=a.conf, imgsz=640, classes=[TL], verbose=False)[0]
    top = 0.0
    for b in r.boxes:
        c = float(b.conf)
        x1, y1, x2, y2 = map(int, b.xyxy[0])
        name, rr, gg = color_of(frame, x1, y1, x2, y2)
        col = {"red": (0, 0, 255), "green": (0, 200, 0)}.get(name, (200, 200, 200))
        cv2.rectangle(frame, (x1, y1), (x2, y2), col, 2)
        cv2.putText(frame, f"{name} {c:.2f} r{rr} g{gg}", (x1, max(y1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
        top = max(top, c)
    cv2.putText(frame, f"traffic light top: {top:.2f} (conf>={a.conf})", (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.imshow("housing only (q to quit)", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
