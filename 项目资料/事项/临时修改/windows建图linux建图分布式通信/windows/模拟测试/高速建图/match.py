"""电脑建图日志（每 5 s 一个 odom 位姿，电脑时间）对车上包（里程计轨迹，车上时间）：
对每个 (趟, 包)：每个日志位姿在包轨迹里找最近点 → 距离、对应的包时间；
时间差（电脑 - 车）应是常数；输出距离中位数、时间差的离散度和匹配上的比例。"""
import glob, json, math, os, sqlite3, struct, datetime, statistics, bisect

BAGS = r'C:\Users\lzc2\Desktop\小车原始包'


def odom(data):
    o = 4
    def al(n):
        nonlocal o
        o += (-(o - 4)) % n
    def u32():
        nonlocal o
        al(4); v = struct.unpack_from('<I', data, o)[0]; o += 4; return v
    def f64(n):
        nonlocal o
        al(8); v = struct.unpack_from('<%dd' % n, data, o); o += 8 * n; return v
    u32(); u32()
    for _ in range(2):
        n = u32(); o += n
    return f64(3)[:2]


bags = {}
for d in sorted(glob.glob(os.path.join(BAGS, 'map_*'))):
    name = os.path.basename(d)
    if os.path.isdir(d + '_修复'):
        continue
    db = glob.glob(os.path.join(d, '*.db3'))[0]
    try:
        con = sqlite3.connect(db)
        tid = [i for i, n in con.execute('select id,name from topics') if n == '/encoder_imu_odom'][0]
    except Exception:
        continue
    P = []
    for t, data in con.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
        try:
            x, y = odom(data); P.append((t / 1e9, x, y))
        except Exception:
            pass
    bags[name.replace('_修复', '')] = P

runs = {}
for d in sorted(glob.glob(r'D:\ubuntu\slam\maps\run_20261103_*')):
    f = os.path.join(d, '.run1-live.jsonl')
    if not os.path.exists(f):
        continue
    S = []
    for line in open(f, encoding='utf-8', errors='replace'):
        try:
            r = json.loads(line)
        except Exception:
            continue
        od = r.get('odom')
        if od and (abs(od[0]) > 0.05 or abs(od[1]) > 0.05):
            S.append((datetime.datetime.fromisoformat(r['time_utc']).timestamp(), od[0], od[1]))
    if len(S) >= 3:
        runs[os.path.basename(d)[4:17]] = S

for rn, S in runs.items():
    res = []
    for bn, P in bags.items():
        if len(P) < 50:
            continue
        # 用网格加速最近点
        best = []
        for ts, x, y in S:
            bd, bt = 1e9, None
            for t, px, py in P[::2]:
                dd = (px - x) ** 2 + (py - y) ** 2
                if dd < bd:
                    bd, bt = dd, t
            best.append((math.sqrt(bd), ts - bt))
        good = [b for b in best if b[0] < 0.15]
        if len(good) < max(3, len(S) // 3):
            continue
        offs = [b[1] for b in good]
        med = statistics.median(offs)
        cons = sum(1 for o in offs if abs(o - med) < 3) / len(S)
        res.append((cons, bn, len(good), len(S), med))
    res.sort(reverse=True)
    top = ' ; '.join(f'{bn} 一致 {c:.0%} ({g}/{n}) 时差 {o/3600:+.3f} h' for c, bn, g, n, o in res[:2])
    print(f'{rn}: {top or "没有匹配的包"}')
