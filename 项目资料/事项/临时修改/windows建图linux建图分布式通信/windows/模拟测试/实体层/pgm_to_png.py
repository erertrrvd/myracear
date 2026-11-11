"""pgm -> 放大的 png。用法：python pgm_to_png.py <in.pgm> <out.png> [放大倍数]"""
import re, struct, sys, zlib
from pathlib import Path
import numpy as np
d = Path(sys.argv[1]).read_bytes()
m = re.match(rb'P5\s+(\d+)\s+(\d+)\s+(\d+)\s', d)
w, h = int(m.group(1)), int(m.group(2))
img = np.frombuffer(d[m.end():m.end() + w * h], np.uint8).reshape(h, w)
k = int(sys.argv[3]) if len(sys.argv) > 3 else 2
img = np.kron(img, np.ones((k, k), np.uint8))
raw = b''.join(b'\x00' + img[r].tobytes() for r in range(img.shape[0]))
ch = lambda t, dd: struct.pack('>I', len(dd)) + t + dd + struct.pack('>I', zlib.crc32(t + dd) & 0xffffffff)
Path(sys.argv[2]).write_bytes(b'\x89PNG\r\n\x1a\n' + ch(b'IHDR', struct.pack('>IIBBBBB', img.shape[1], img.shape[0], 8, 0, 0, 0, 0))
                              + ch(b'IDAT', zlib.compress(raw, 9)) + ch(b'IEND', b''))
print(w, h)
