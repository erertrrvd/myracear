"""把 run1/map.pgm 和 map_solid.pgm 的同一段（x 从 X0 到 X1 米）放大后上下拼成一张 PNG。
用法：python compare_png.py <run1目录> <输出png> [X0] [X1] [放大倍数]"""
import re, struct, sys, zlib
from pathlib import Path
import numpy as np


def load(run, name):
    y = (run / f'{name}.yaml').read_text(encoding='utf-8')
    res = float(re.search(r'resolution:\s*(\S+)', y).group(1))
    ox = float(re.search(r'origin:\s*\[([^\]]+)\]', y).group(1).split(',')[0])
    d = (run / f'{name}.pgm').read_bytes()
    m = re.match(rb'P5\s+(\d+)\s+(\d+)\s+(\d+)\s', d)
    w, h = int(m.group(1)), int(m.group(2))
    return np.frombuffer(d[m.end():m.end() + w * h], np.uint8).reshape(h, w), res, ox


def write_png(path, img):
    h, w = img.shape
    raw = b''.join(b'\x00' + img[r].tobytes() for r in range(h))
    ch = lambda t, d: struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    path.write_bytes(b'\x89PNG\r\n\x1a\n' + ch(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0))
                     + ch(b'IDAT', zlib.compress(raw, 9)) + ch(b'IEND', b''))


run = Path(sys.argv[1])
x0, x1 = (float(sys.argv[3]), float(sys.argv[4])) if len(sys.argv) > 4 else (0.0, 20.0)
k = int(sys.argv[5]) if len(sys.argv) > 5 else 4
parts = []
for name in ('map', 'map_solid'):
    img, res, ox = load(run, name)
    a, b = int((x0 - ox) / res), int((x1 - ox) / res)
    crop = img[:, max(a, 0):b]
    big = np.kron(crop, np.ones((k, k), np.uint8))
    parts += [big, np.full((6, big.shape[1]), 128, np.uint8)]
write_png(Path(sys.argv[2]), np.vstack(parts[:-1]))
print('ok', sys.argv[2])
