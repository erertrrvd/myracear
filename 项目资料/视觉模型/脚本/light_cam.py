r"""笔记本摄像头试验：专用红绿灯模型 traffic-lights.pt（YOLOv8，8 类，只有红/绿）
运行：python 脚本\light_cam.py
按 q 退出。--conf 可调阈值，默认 0.15。
"""
import argparse
import os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # 视觉模型 文件夹
import cv2
from ultralytics import YOLO

ap = argparse.ArgumentParser()
ap.add_argument("--model", default=os.path.join(ROOT, "模型", "通用模型试验", "traffic-lights.pt"))
ap.add_argument("--conf", type=float, default=0.4)
ap.add_argument("--cam", type=int, default=0)
a = ap.parse_args()

model = YOLO(a.model)
cap = cv2.VideoCapture(a.cam, cv2.CAP_DSHOW)
if not cap.isOpened():
    raise SystemExit("打不开摄像头")

while True:
    ok, frame = cap.read()
    if not ok:
        break
    r = model.predict(frame, conf=a.conf, imgsz=640, verbose=False)[0]
    best = "none"
    best_c = 0.0
    for b in r.boxes:
        name = model.names[int(b.cls)]
        c = float(b.conf)
        x1, y1, x2, y2 = map(int, b.xyxy[0])
        col = (0, 0, 255) if name.startswith("Red") else (0, 200, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), col, 2)
        cv2.putText(frame, f"{name} {c:.2f}", (x1, max(y1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
        if c > best_c:
            best, best_c = name, c
    cv2.putText(frame, f"top: {best} {best_c:.2f} (conf>={a.conf})", (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.imshow("traffic-lights.pt (q to quit)", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
