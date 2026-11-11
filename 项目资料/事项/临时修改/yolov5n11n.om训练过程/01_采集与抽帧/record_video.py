#!/usr/bin/env python3
"""直接读 USB 摄像头录视频，不依赖 ROS 话题，用来采集红绿灯训练素材。

用法（在小车上）：
  cd /home/racecar/src/my_racecar/camera_capture
  /usr/bin/python3 record_video.py --check                            # 只检查摄像头能不能用，不录
  /usr/bin/python3 record_video.py --name red_near --seconds 20
  /usr/bin/python3 record_video.py --name green_far                   # 不限时，Ctrl+C 或 q 结束

输出：camera_capture/videos/<name>_<时间>.mp4（不会覆盖旧文件）
  --show  弹出预览窗口（需要 MobaXterm X server），默认不弹，避免拖慢
"""
import argparse
import os
import time

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="clip")
    ap.add_argument("--seconds", type=float, default=0, help="0 = 不限时")
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--out-dir", default=os.path.join(HERE, "videos"))
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--check", action="store_true", help="只检查摄像头，不录视频")
    a = ap.parse_args()

    os.makedirs(a.out_dir, exist_ok=True)
    cap = cv2.VideoCapture(a.cam)
    if not cap.isOpened():
        raise SystemExit("无法打开摄像头（可能被别的程序占用）")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)

    if a.check:
        t0, n = time.time(), 0
        while time.time() - t0 < 3:
            ok, frame = cap.read()
            if not ok:
                raise SystemExit("摄像头能打开，但读不到画面")
            n += 1
        h, w = frame.shape[:2]
        print(f"摄像头正常：{w}x{h}，3 秒读到 {n} 帧，约 {n / 3:.1f} fps")
        cap.release()
        return

    ok, frame = cap.read()
    if not ok:
        raise SystemExit("读不到画面")
    h, w = frame.shape[:2]
    path = os.path.join(a.out_dir, f"{a.name}_{time.strftime('%Y%m%d_%H%M%S')}.mp4")
    out = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (w, h))
    print(f"开始录制 {w}x{h}，保存到 {path}，Ctrl+C 结束")

    t0 = time.time()
    n = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("读帧失败，结束")
                break
            out.write(frame)
            n += 1
            if a.show:
                cv2.imshow("record (q to stop)", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            if a.seconds and time.time() - t0 >= a.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        out.release()
        cv2.destroyAllWindows()
    dt = time.time() - t0
    print(f"录制结束：{n} 帧，{dt:.1f} 秒，实际约 {n / dt:.1f} fps -> {path}")


if __name__ == "__main__":
    main()
