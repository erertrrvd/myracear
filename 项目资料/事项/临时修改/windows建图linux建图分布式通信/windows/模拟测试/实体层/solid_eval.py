"""对比 run1/map.pgm 和 map_solid.pgm：锥桶找回、走动物体残留、半径。用法：python solid_eval.py <run1目录> [png输出目录]"""
import json, re, sys
from pathlib import Path
import numpy as np

START_X = 1.0                                   # fake_car3 从 x=1 出发，SLAM 地图以出发点为原点
CONES = [(3.0 + 3.0 * i - START_X, 1.2 if i % 2 == 0 else -1.2) for i in range(12)]
CONE_R = 0.08
MOVER_X = 20.0 - START_X


def load(run, name):
    y = (run / f'{name}.yaml').read_text(encoding='utf-8')
    res = float(re.search(r'resolution:\s*(\S+)', y).group(1))
    ox, oy = [float(v) for v in re.search(r'origin:\s*\[([^\]]+)\]', y).group(1).split(',')[:2]]
    d = (run / f'{name}.pgm').read_bytes()
    m = re.match(rb'P5\s+(\d+)\s+(\d+)\s+(\d+)\s', d)
    w, h = int(m.group(1)), int(m.group(2))
    img = np.frombuffer(d[m.end():m.end() + w * h], np.uint8).reshape(h, w)[::-1]   # 行 0 = y 最小
    return img, res, ox, oy


def stats(img, res, ox, oy):
    occ = img < 50
    jj, ii = np.nonzero(occ)
    x, y = ox + (ii + 0.5) * res, oy + (jj + 0.5) * res
    found, cells = 0, []
    for cx, cy in CONES:
        n = int(np.sum(np.hypot(x - cx, y - cy) < CONE_R + 0.12))
        cells.append(n)
        found += n > 0
    near_cone = np.zeros_like(x, bool)
    for cx, cy in CONES:
        near_cone |= np.hypot(x - cx, y - cy) < 0.9     # 排除真锥桶（含 SLAM 偏移）附近
    mover = int(np.sum((np.abs(x - MOVER_X) < 0.6) & (np.abs(y) < 1.7) & ~near_cone))
    return found, cells, mover


run = Path(sys.argv[1])
for name in ('map', 'map_solid'):
    img, res, ox, oy = load(run, name)
    f, cells, mover = stats(img, res, ox, oy)
    print(f'{name:10s} 锥桶找回 {f}/12  每个锥桶的障碍格 {cells}  走动物体区域障碍格 {mover}')
    if len(sys.argv) > 2:
        import pymupdf  # noqa  只为确认环境；PNG 直接复制
sj = run / 'solid.json'
if sj.is_file():
    d = json.loads(sj.read_text(encoding='utf-8'))
    print('solid.json:', {k: v for k, v in d.items() if k != 'cone_centers'})
    errs = []
    for cx, cy in CONES:
        if d['cone_centers']:
            e = min(np.hypot(px - cx, py - cy) for px, py, *_ in d['cone_centers'])
            errs.append(round(float(e), 3))
    print('每个真锥桶到最近画出圆心的距离(m):', errs)
    print('画出的锥桶对上真锥桶（0.8 m 内，SLAM 本身随车速有 0.2~0.7 m 整体偏移）:', sum(e < 0.8 for e in errs), '/ 12')
    extra = [c for c in d['cone_centers'] if min(np.hypot(c[0] - a, c[1] - b) for a, b in CONES) > 0.8]
    print('多画的圆 [x, y, 原图团格数]:', extra)
