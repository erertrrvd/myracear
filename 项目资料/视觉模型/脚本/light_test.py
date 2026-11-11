"""笔记本摄像头试验：YOLOv8n(COCO) 找 traffic light 框，再用 HSV 判红/绿。
用法：python light_test.py            # 摄像头，按 q 退出
      python light_test.py --image a.jpg
"""
import argparse
import os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # 视觉模型 文件夹
import cv2
import numpy as np

TL = 9  # COCO 类别号：traffic light
SZ = 640


def letterbox(img):
    h, w = img.shape[:2]
    r = SZ / max(h, w)
    nh, nw = int(round(h * r)), int(round(w * r))
    out = np.full((SZ, SZ, 3), 114, np.uint8)
    top, left = (SZ - nh) // 2, (SZ - nw) // 2
    out[top:top + nh, left:left + nw] = cv2.resize(img, (nw, nh))
    return out, r, left, top


def detect(net, frame, conf=0.15):
    img, r, left, top = letterbox(frame)
    net.setInput(cv2.dnn.blobFromImage(img, 1 / 255.0, (SZ, SZ), swapRB=True))
    out = net.forward()[0]
    if out.shape[0] < out.shape[1]:  # (84, 8400) -> (8400, 84)
        out = out.T
    scores = out[:, 4 + TL]
    keep = scores > conf
    boxes, confs = [], []
    for row, s in zip(out[keep], scores[keep]):
        cx, cy, w, h = row[:4]
        x = (cx - w / 2 - left) / r
        y = (cy - h / 2 - top) / r
        boxes.append([int(x), int(y), int(w / r), int(h / r)])
        confs.append(float(s))
    idx = cv2.dnn.NMSBoxes(boxes, confs, conf, 0.45)
    return [(boxes[i], confs[i]) for i in np.array(idx).flatten()] if len(boxes) else []


def color_of(frame, box):
    x, y, w, h = box
    x, y = max(x, 0), max(y, 0)
    crop = frame[y:y + h, x:x + w]
    if crop.size == 0:
        return "none", 0, 0
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hh, ss, vv = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    lit = (ss > 80) & (vv > 150)
    red = int((lit & ((hh < 10) | (hh > 170))).sum())
    green = int((lit & (hh > 40) & (hh < 90)).sum())
    if max(red, green) < 8:
        return "none", red, green
    return ("red" if red > green else "green"), red, green


def annotate(net, frame):
    for box, c in detect(net, frame):
        name, r, g = color_of(frame, box)
        col = {"red": (0, 0, 255), "green": (0, 200, 0)}.get(name, (200, 200, 200))
        x, y, w, h = box
        cv2.rectangle(frame, (x, y), (x + w, y + h), col, 2)
        cv2.putText(frame, f"{name} {c:.2f} r{r} g{g}", (x, max(y - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
    return frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image")
    ap.add_argument("--model", default=os.path.join(ROOT, "模型", "通用模型试验", "yolov8n.onnx"))
    ap.add_argument("--cam", type=int, default=0)
    a = ap.parse_args()
    net = cv2.dnn.readNetFromONNX(a.model)
    if a.image:
        f = annotate(net, cv2.imread(a.image))
        cv2.imwrite(a.image + ".out.jpg", f)
        print("saved", a.image + ".out.jpg")
        return
    cap = cv2.VideoCapture(a.cam, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("打不开摄像头")
        return
    while True:
        ok, f = cap.read()
        if not ok:
            break
        cv2.imshow("light test (q to quit)", annotate(net, f))
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
