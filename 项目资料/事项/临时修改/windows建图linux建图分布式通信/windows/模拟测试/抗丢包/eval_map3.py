"""评估一张 autosave 地图：6 个锥桶找回几个、锥桶占据格数、走动的人留下多少痕迹。
用法：python eval_map.py <map.yaml>   （地图坐标 = 世界坐标 - (START_X, 0)）"""
import re, sys
from PIL import Image

CONES = [(3.0 + 3.0 * i, 1.2 if i % 2 == 0 else -1.2) for i in range(12)]
START_X = 1.0
MOVER_BOX = (19.6, 20.4, -1.75, 1.75)        # 走动的人活动的区域（世界坐标 x0,x1,y0,y1）

yml = open(sys.argv[1], encoding='utf-8').read()
res = float(re.search(r'resolution:\s*(\S+)', yml).group(1))
ox, oy = [float(v) for v in re.search(r'origin:\s*\[([^\]]+)\]', yml).group(1).split(',')[:2]]
im = Image.open(sys.argv[1].replace('map.yaml', 'map.pgm')).convert('L')
w, h = im.size; px = im.load()


def occ_cells():
    out = []
    for gy in range(h):
        for gx in range(w):
            if px[gx, h - 1 - gy] < 80:          # pgm 第 0 行是栅格最大 y
                out.append((ox + (gx + 0.5) * res + START_X, oy + (gy + 0.5) * res))   # 转回世界坐标
    return out


cells = occ_cells()
found = 0; cone_cells = 0
for cx, cy in CONES:
    n = sum(1 for x, y in cells if (x - cx) ** 2 + (y - cy) ** 2 <= 0.2 ** 2)
    cone_cells += n; found += 1 if n else 0
x0, x1, y0, y1 = MOVER_BOX
trace = sum(1 for x, y in cells if x0 <= x <= x1 and y0 <= y <= y1)
print(f'锥桶找回 {found}/12，锥桶附近占据格 {cone_cells}，走动区域残留占据格 {trace}，总占据格 {len(cells)}')
