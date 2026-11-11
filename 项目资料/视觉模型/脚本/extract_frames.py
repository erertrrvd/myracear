r"""视频抽帧。
用法：python 脚本\extract_frames.py 视频路径 [--out 输出文件夹] [--every 5] [--start 0]
  --every  每几帧取一张（30fps 下 5 约每 0.17 秒一张）
  --start  图片起始编号，接着上次的编号往后存，避免覆盖
输出：frame_0000.jpg ...（沿用课程命名）
"""
import argparse
import os
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("video")
ap.add_argument("--out", default=None)
ap.add_argument("--every", type=int, default=5)
ap.add_argument("--start", type=int, default=0)
a = ap.parse_args()

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # 视觉模型 文件夹
out = a.out or os.path.join(ROOT, "数据", "抽帧图片", os.path.splitext(os.path.basename(a.video))[0])
os.makedirs(out, exist_ok=True)

cap = cv2.VideoCapture(a.video)
if not cap.isOpened():
    raise SystemExit("无法打开视频文件：" + a.video)
fps = cap.get(cv2.CAP_PROP_FPS)
total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
print(f"帧率 {fps:.1f} FPS，总帧数 {total}，时长 {total / fps:.1f} 秒")

n, saved = 0, 0
while True:
    ok, frame = cap.read()
    if not ok:
        break
    if n % a.every == 0:
        cv2.imwrite(os.path.join(out, f"frame_{a.start + saved:04d}.jpg"), frame)
        saved += 1
    n += 1
cap.release()
print(f"共保存 {saved} 张到 {out}")
