"""每趟汇总：车动时段里 odom_pubs=2 的占比、里程计 5 s 位移推出的“假车速”、corr 5 s 最大跳、里程计条数/s。"""
import json, sys, math, glob, os
for path in sorted(glob.glob(r'D:\ubuntu\slam\maps\run_202611*\.run1-live.jsonl')):
    rows = []
    for line in open(path, encoding='utf8', errors='replace'):
        try: e = json.loads(line)
        except Exception: continue
        if e.get('event') == 'status' and e.get('odom'): rows.append(e)
    mv = p2 = 0; vmax2 = vmax1 = 0; cj2 = cj1 = 0; r2 = []; r1 = []
    for a, b in zip(rows, rows[1:]):
        do = math.hypot(b['odom'][0] - a['odom'][0], b['odom'][1] - a['odom'][1])
        if do < 0.05: continue          # 车没动
        mv += 1
        v = do / 5.0
        cj = math.hypot(b['corr'][0] - a['corr'][0], b['corr'][1] - a['corr'][1])
        n = (b['odometry_messages'] - a['odometry_messages']) / 5.0
        two = (b.get('link') or {}).get('odom_pubs') == 2
        if two: p2 += 1; vmax2 = max(vmax2, v); cj2 = max(cj2, cj); r2.append(n)
        else: vmax1 = max(vmax1, v); cj1 = max(cj1, cj); r1.append(n)
    if not mv: continue
    name = os.path.basename(os.path.dirname(path))[4:17]
    avg = lambda l: sum(l) / len(l) if l else 0
    print('%s 车动 %3d 段 | 两个里程计 %3d 段: 假车速最大 %5.2f m/s, corr 跳最大 %5.2f m, 里程计 %4.0f 条/s | 一个: 车速最大 %4.2f, corr 跳最大 %5.2f m, %4.0f 条/s' % (
        name, mv, p2, vmax2, cj2, avg(r2), vmax1, cj1, avg(r1)))
