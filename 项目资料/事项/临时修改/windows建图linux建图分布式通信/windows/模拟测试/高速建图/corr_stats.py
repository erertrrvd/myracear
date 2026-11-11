import json, math, sys, statistics as st
for d in sys.argv[1:]:
    rows = [json.loads(l) for l in open(d + '/run1/run.jsonl', encoding='utf-8')]
    rows = [r for r in rows if r.get('corr') and r.get('odom')]
    dm, dd, jumps = [], [], 0
    prev = None
    for r in rows:
        c = r['corr']
        if prev:
            m = math.hypot(c[0] - prev[0], c[1] - prev[1])
            a = abs((c[2] - prev[2] + 180) % 360 - 180)
            dm.append(m); dd.append(a)
            if m > 1.0 or a > 10: jumps += 1
        prev = c
    last = rows[-1]['corr']
    print(f'{d.split("/")[-1]:24s} 5s数 {len(dd):3d}  突跳 {jumps}  |Δ朝向| 均 {st.mean(dd):.2f}° 最大 {max(dd):.1f}°  |Δ位置| 均 {st.mean(dm):.2f} m 最大 {max(dm):.2f} m  结束corr {math.hypot(last[0], last[1]):.2f} m / {last[2]:.1f}°')
