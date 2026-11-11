"""离线跑 solidify 并输出彩色差异图：灰度底 = 原图，红 = 实体层新增，蓝 = 被清掉的原图障碍。
用法：python solid_diff.py <obstacle_layer路径> <map.pgm> <输出png> <半径> [放大倍数] [edge_margin]"""
import ast, math, re, struct, sys, zlib
from pathlib import Path
import numpy as np

src = Path(sys.argv[1]).read_text(encoding='utf-8')
ns = {'np': np, 'math': math}
body = [n for n in ast.parse(src).body if isinstance(n, ast.FunctionDef) and n.name in ('components', 'solidify', 'outside_unknown')]
exec(compile(ast.Module(body, []), 'obstacle_layer', 'exec'), ns)
d = Path(sys.argv[2]).read_bytes()
m = re.match(rb'P5\s+(\d+)\s+(\d+)\s+(\d+)\s', d)
w, h = int(m.group(1)), int(m.group(2))
pgm = np.frombuffer(d[m.end():m.end() + w * h], np.uint8).reshape(h, w)
grid = np.where(pgm < 50, 100, np.where(pgm > 250, 0, -1)).astype(np.int8)[::-1]
import json
kw = json.loads(sys.argv[6]) if len(sys.argv) > 6 else {}
import time; t0 = time.time()
out, cones, rej = ns['solidify'](grid, 0.05, float(sys.argv[4]), **kw)
print(f'用时 {time.time() - t0:.2f} s')
print(f'锥桶 {len(cones)}，没加工的小团 {rej}')
a, b = pgm, np.where(out == 100, 0, np.where(out == 0, 254, 205)).astype(np.uint8)[::-1]
rgb = np.stack([a, a, a], -1).astype(np.uint8)
rgb[(b < 50) & (a >= 50)] = (230, 30, 30)
rgb[(a < 50) & (b >= 50)] = (40, 90, 255)
k = int(sys.argv[5]) if len(sys.argv) > 5 else 3
img = np.kron(rgb, np.ones((k, k, 1), np.uint8))
raw = b''.join(b'\x00' + img[r].tobytes() for r in range(img.shape[0]))
ch = lambda t, dd: struct.pack('>I', len(dd)) + t + dd + struct.pack('>I', zlib.crc32(t + dd) & 0xffffffff)
Path(sys.argv[3]).write_bytes(b'\x89PNG\r\n\x1a\n' + ch(b'IHDR', struct.pack('>IIBBBBB', img.shape[1], img.shape[0], 8, 2, 0, 0, 0))
                              + ch(b'IDAT', zlib.compress(raw, 9)) + ch(b'IEND', b''))
