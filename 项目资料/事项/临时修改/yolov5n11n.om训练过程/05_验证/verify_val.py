#!/usr/bin/env python3
"""在验证集上跑模型（.onnx 或 .om），把判断的状态和标注文件里的真实状态对比，打印混淆矩阵。
真实状态由标签文件推出：有 class 1 -> red，有 class 2 -> green，只有 class 0 -> off，没有框 -> none。
和 light_infer.py 放在同一目录，复用它的识别和判断逻辑。
用法：
  D:/light_ai/venv/Scripts/python.exe verify_val.py --model ../06_模型文件/yolov5n_best.onnx
  （在小车上跑 .om：source CANN 环境后 python3 verify_val.py --model output/yolov5n_bs1.om --images 图片目录 --labels 标签目录）
"""
import argparse
import glob
import os
import sys
import types
from collections import Counter

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import light_infer as li

DS = "D:/VSCode/ros2技术集/智能车总文件/视觉模型/数据/数据集打包_给训练电脑用/light_dataset"
ap = argparse.ArgumentParser()
ap.add_argument("--model", required=True)
ap.add_argument("--images", default=DS + "/images/val")
ap.add_argument("--labels", default=DS + "/labels/val")
ap.add_argument("--conf-box", type=float, default=0.5)
ap.add_argument("--conf-lamp", type=float, default=0.5)
a = ap.parse_args()
dec = types.SimpleNamespace(conf_box=a.conf_box, conf_lamp=a.conf_lamp, ar_min=2.5, ar_max=4.5,
                            cy_min=0.20, cy_max=0.85, no_filter=False)


def truth(stem):
    p = os.path.join(a.labels, stem + ".txt")
    cls = {ln.split()[0] for ln in open(p) if ln.strip()} if os.path.exists(p) else set()
    return "red" if "1" in cls else "green" if "2" in cls else "off" if "0" in cls else "none"


be = li.Backend(a.model)
cm, bad, n = Counter(), [], 0
for f in sorted(glob.glob(os.path.join(a.images, "*.jpg"))):
    img = cv2.imdecode(np.fromfile(f, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        continue
    H, W = img.shape[:2]
    raw, _, _ = li.decide(li.detect(be, img), W, H, dec)
    t = truth(os.path.splitext(os.path.basename(f))[0])
    cm[(t, raw)] += 1
    n += 1
    if t != raw:
        bad.append((os.path.basename(f), t, raw))
print(f"共 {n} 张，错误 {len(bad)} 张")
for (t, r), c in sorted(cm.items()):
    print(f"  真实 {t:6s} -> 判断 {r:8s}: {c}")
for b in bad[:20]:
    print("  判错：", b)
print("红判绿（最危险）:", cm.get(("red", "green"), 0))
