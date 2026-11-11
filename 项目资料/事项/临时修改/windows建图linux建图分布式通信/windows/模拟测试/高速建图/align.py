import math, statistics, bisect, sys
sys.argv = ['x']
exec(open('match.py', encoding='utf-8').read().split('for rn, S in runs.items():')[0])
def interp(P, T, t):
    i = bisect.bisect_left(T, t)
    if i <= 0 or i >= len(P): return None
    a, b = P[i-1], P[i]
    if b[0] - a[0] > 1.0: return None
    f = (t - a[0]) / (b[0] - a[0]); return a[1] + f*(b[1]-a[1]), a[2] + f*(b[2]-a[2])
TT = {bn: [p[0] for p in P] for bn, P in bags.items()}
for rn, S in runs.items():
    out = []
    for bn, P in bags.items():
        if len(P) < 50: continue
        T = TT[bn]
        lo, hi = S[0][0] - T[-1], S[-1][0] - T[0]
        best = None
        o = lo
        while o <= hi:
            ds = []
            for ts, x, y in S:
                q = interp(P, T, ts - o)
                if q: ds.append(math.hypot(q[0]-x, q[1]-y))
            if len(ds) >= max(4, len(S)//2):
                sc = statistics.median(ds)
                if best is None or sc < best[0]: best = (sc, o, len(ds))
            o += 0.5
        if best: out.append((best[0], bn, best[2], len(S)))
    out.sort()
    print(rn, ' ; '.join(f'{bn} 中位误差 {e:.2f} m ({k}/{n})' for e, bn, k, n in out[:2]))
