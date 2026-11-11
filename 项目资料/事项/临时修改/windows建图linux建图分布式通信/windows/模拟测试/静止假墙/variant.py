import sys, re
from pathlib import Path
from collections import Counter
import numpy as np
src = Path(sys.argv[1]).read_text(encoding='utf-8')
body = src[src.index('def yaw_of'):src.index('class Track')]
body = body.replace("rejected += 1                       # 靠边", "rejected += 1; REASONS.append('edge')  #")
body = body.replace("rejected += 1                       # 外环有别的团", "rejected += 1; REASONS.append('ring_other')  #")
body = body.replace("rejected += 1                       # 周围大多没探明", "rejected += 1; REASONS.append('ring_free')  #")
# 变体：边缘只算墙（不算和地图边界连通的未知），不因靠近地图边界拒收
v = body.replace("edge = outside.copy()", "edge = np.zeros_like(outside) if NOFRONT else outside.copy()")
v = v.replace("if edge[ej[m], ei[m]].any() or not m.all():", "if edge[ej[m], ei[m]].any() or (not m.all() and not NOFRONT):")
d = Path(sys.argv[2]); raw=(d/'map.pgm').read_bytes(); p=raw.split(maxsplit=4); w,h=int(p[1]),int(p[2])
img=np.frombuffer(p[4][-w*h:],np.uint8).reshape(h,w)[::-1]
grid=np.full(img.shape,-1,np.int8); grid[img<100]=100; grid[img>250]=0
for nof in (False, True):
    for rf in (0.6, 0.3, 0.0):
        g={'np':np,'math':__import__('math'),'REASONS':[],'NOFRONT':nof}; exec(v,g)
        out,cones,rej=g['solidify'](grid,0.05,0.089,ring_free_min=rf)
        print(f'不算视野边缘={nof} ring_free_min={rf}: 锥桶 {len(cones)}，拒收 {dict(Counter(g["REASONS"]))}')
