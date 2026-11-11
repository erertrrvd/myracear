"""逐个点团统计 obstacle_layer.solidify 拒收的原因。用法：python why_rejected.py <obstacle_layer> <map_dir>"""
import sys, re
from pathlib import Path
import numpy as np

src = Path(sys.argv[1]).read_text(encoding='utf-8')
body = src[src.index('def yaw_of'):src.index('class Track')]
# 在每个 rejected += 1 处记录原因
body = body.replace("rejected += 1                       # 靠边", "rejected += 1; REASONS.append(('edge', ci, cj))  # 靠边")
body = body.replace("rejected += 1                       # 外环有别的团", "rejected += 1; REASONS.append(('ring_other', ci, cj))  #")
body = body.replace("rejected += 1                       # 周围大多没探明", "rejected += 1; REASONS.append(('ring_free', ci, cj))  #")
g = {'np': np, 'math': __import__('math'), 'REASONS': []}
exec(body, g)

d = Path(sys.argv[2])
y = dict(re.findall(r'^(\w+):\s*(.+)$', (d / 'map.yaml').read_text(), re.M))
res = float(y['resolution'])
raw = (d / 'map.pgm').read_bytes()
parts = raw.split(maxsplit=4)
w, h = int(parts[1]), int(parts[2])
img = np.frombuffer(parts[4][-w * h:], np.uint8).reshape(h, w)[::-1]
grid = np.full(img.shape, -1, np.int8)
grid[img < 100] = 100
grid[img > 250] = 0
print('map', w, h, 'res', res, 'occ', int((grid == 100).sum()), 'free', int((grid == 0).sum()))
out, cones, rej = g['solidify'](grid, res, 0.089)
print('cones', len(cones), 'rejected', rej)
from collections import Counter
print(Counter(r[0] for r in g['REASONS']))
