"""查指定像素位置（差异图里的坐标 / 放大倍数）附近：到房间外未知、到各尺寸障碍团的最近距离。"""
import ast, math, re, sys
from pathlib import Path
import numpy as np
src = Path(sys.argv[1]).read_text(encoding='utf-8')
ns = {'np': np, 'math': math}
body = [n for n in ast.parse(src).body if isinstance(n, ast.FunctionDef) and n.name in ('components', 'outside_unknown')]
exec(compile(ast.Module(body, []), 'x', 'exec'), ns)
d = Path(sys.argv[2]).read_bytes()
m = re.match(rb'P5\s+(\d+)\s+(\d+)\s+(\d+)\s', d)
w, h = int(m.group(1)), int(m.group(2))
pgm = np.frombuffer(d[m.end():m.end() + w * h], np.uint8).reshape(h, w)   # 行 0 = 图片顶部
unk = pgm == 205
out = ns['outside_unknown'](unk)
occ = pgm < 50
comps = ns['components'](occ)
size = lambda c: (max(c[0].max() - c[0].min(), c[1].max() - c[1].min()) + 1) * 0.05
k = 3
yy, xx = np.mgrid[0:h, 0:w]
for px, py in [tuple(map(int, s.split(','))) for s in sys.argv[3:]]:
    i, j = px // k, py // k
    dist = lambda mask: (np.hypot(xx[mask] - i, yy[mask] - j).min() * 0.05) if mask.any() else float('inf')
    near = sorted(((float(np.hypot(c[1] - i, c[0] - j).min() * 0.05), round(size(c), 2)) for c in comps), key=lambda t: t[0])
    near = [t for t in near if t[0] > 0.12][:4]
    print(f'({px},{py}) 到房间外未知 {dist(out):.2f} m，到任何未知 {dist(unk):.2f} m；最近的其他障碍团 (距离 m, 尺寸 m): {near}')
