r"""只开摄像头预览，不做任何识别。按 q 退出。
运行：python 脚本\cam_only.py [--cam 0]
"""
import argparse
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("--cam", type=int, default=0)
a = ap.parse_args()

cap = cv2.VideoCapture(a.cam, cv2.CAP_DSHOW)
if not cap.isOpened():
    raise SystemExit("打不开摄像头")
while True:
    ok, f = cap.read()
    if not ok:
        break
    cv2.imshow("camera only (q to quit)", f)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
