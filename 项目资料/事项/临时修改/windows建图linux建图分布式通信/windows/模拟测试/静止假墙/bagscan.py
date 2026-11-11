"""读 rosbag2 (sqlite3) 里的 /scan，统计远处点。用法：python bagscan.py <db3>"""
import sqlite3, struct, sys
import numpy as np


def parse_scan(b):
    o = 4
    sec, nsec = struct.unpack_from('<iI', b, o); o += 8
    n = struct.unpack_from('<I', b, o)[0]; o += 4
    frame = b[o:o + n - 1].decode(); o += n
    o = (o + 3) & ~3
    amin, amax, ainc, tinc, stime, rmin, rmax = struct.unpack_from('<7f', b, o); o += 28
    n = struct.unpack_from('<I', b, o)[0]; o += 4
    r = np.frombuffer(b, '<f4', n, o).copy(); o += 4 * n
    m = struct.unpack_from('<I', b, o)[0]; o += 4
    it = np.frombuffer(b, '<f4', m, o).copy()
    return dict(t=sec + nsec * 1e-9, frame=frame, amin=amin, amax=amax, ainc=ainc, rmin=rmin, rmax=rmax, r=r, i=it)


db = sqlite3.connect(sys.argv[1])
tid = db.execute("select id from topics where name='/scan'").fetchone()[0]
scans = [parse_scan(row[0]) for row in db.execute('select data from messages where topic_id=? order by timestamp', (tid,))]
s0 = scans[0]
print('frames', len(scans), 'frame_id', s0['frame'], 'amin/amax/inc', s0['amin'], s0['amax'], s0['ainc'],
      'range_min/max', s0['rmin'], s0['rmax'], 'n', len(s0['r']), 'intensities', len(s0['i']))
print('点数分布', np.unique([len(s['r']) for s in scans], return_counts=True))
allr = np.concatenate([s['r'] for s in scans])
fin = allr[np.isfinite(allr)]
print('inf/nan 比例', round(1 - len(fin) / len(allr), 3), '有限值 max', fin.max())
print('距离直方图 (0.5 m 一格):')
h, e = np.histogram(fin, bins=np.arange(0, np.ceil(fin.max()) + 0.5, 0.5))
for k in range(len(h)):
    if h[k]:
        print(f'  {e[k]:4.1f}-{e[k+1]:4.1f} m: {h[k]}')
# 出现最多的具体数值（是否有固定值）
vals, cnt = np.unique(np.round(fin, 3), return_counts=True)
top = np.argsort(cnt)[-8:][::-1]
print('最常见数值', [(float(vals[k]), int(cnt[k])) for k in top])
# 用同一点数的帧，逐角度看稳定性
n0 = len(s0['r'])
same = np.array([s['r'] for s in scans if len(s['r']) == n0])
ok = np.isfinite(same) & (same >= s0['rmin']) & (same <= s0['rmax'])
pres = ok.mean(axis=0)
med = np.array([np.median(same[ok[:, k], k]) if ok[:, k].any() else np.nan for k in range(n0)])
sd = np.array([np.std(same[ok[:, k], k]) if ok[:, k].sum() > 1 else np.nan for k in range(n0)])
print('逐角度：回波出现率 >=0.9 的角度数', int((pres >= 0.9).sum()), '/', n0)
far = (pres >= 0.9) & (med > 3.0)
print('稳定且 >3 m 的角度数', int(far.sum()), ' 中位距离分布', np.percentile(med[far], [0, 25, 50, 75, 100]).round(2) if far.any() else None,
      ' 帧间标准差中位', np.nanmedian(sd[far]).round(3) if far.any() else None)
if len(s0['i']):
    alli = np.array([s['i'] for s in scans if len(s['i']) == n0])
    mi = np.nanmedian(np.where(ok, alli, np.nan), axis=0)
    for lo, hi in ((0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 8), (8, 30)):
        m = (pres >= 0.9) & (med >= lo) & (med < hi)
        if m.any():
            print(f'  {lo}-{hi} m: 角度 {int(m.sum())}，强度中位 {np.nanmedian(mi[m]):.1f}，帧间距离标准差中位 {np.nanmedian(sd[m]):.3f}')
np.savez('scanstats.npz', med=med, pres=pres, sd=sd, amin=s0['amin'], ainc=(s0['amax'] - s0['amin']) / (n0 - 1))
