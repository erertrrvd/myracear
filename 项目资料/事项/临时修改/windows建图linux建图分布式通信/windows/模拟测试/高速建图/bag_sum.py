"""每个包一行摘要：时长、scan/odom 条数、里程、最高车速、两个里程计迹象；损坏的 db3 按 rowid 逐条抢救。"""
import glob, math, os, sqlite3, struct, sys


def rows_safe(con, sql_ids, tid):
    """按 rowid 分段读，跳过坏页。返回 [(ts, data)]。"""
    out = []
    try:
        lo, hi = con.execute("select min(id), max(id) from messages").fetchone()
    except sqlite3.DatabaseError:
        lo, hi = 1, 2_000_000
    if lo is None:
        return out
    step = 500
    i = lo
    while i <= hi:
        try:
            out += con.execute("select timestamp, data from messages where topic_id=? and id>=? and id<?",
                               (tid, i, i + step)).fetchall()
        except sqlite3.DatabaseError:
            for j in range(i, i + step):
                try:
                    out += con.execute("select timestamp, data from messages where topic_id=? and id=?", (tid, j)).fetchall()
                except sqlite3.DatabaseError:
                    pass
        i += step
    out.sort()
    return out


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
    sec = u32(); nsec = u32()
    for _ in range(2):
        n = u32(); o += n
    px, py, pz = f64(3); qx, qy, qz, qw = f64(4); f64(36)
    vx, vy, vz = f64(3)
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return px, py, yaw, vx


for d in sys.argv[1:]:
    name = os.path.basename(d.rstrip('\\/'))
    db = sorted(glob.glob(os.path.join(d, '*.db3')))[0]
    try:
        con = sqlite3.connect(db)
        topics = {n: i for i, n in con.execute('select id, name from topics')}
    except sqlite3.DatabaseError as e:
        print(f'{name}: 打不开 topics 表（{e}）'); continue
    oid = topics.get('/encoder_imu_odom') or topics.get('/key1_odom')
    sid = topics.get('/scan')
    sc = rows_safe(con, None, sid) if sid else []
    od = rows_safe(con, None, oid) if oid else []
    P = []
    for t, data in od:
        try:
            P.append((t / 1e9,) + odom(data))
        except Exception:
            pass
    if not sc:
        print(f'{name}: 没有 /scan'); continue
    t0, t1 = sc[0][0] / 1e9, sc[-1][0] / 1e9
    dur = t1 - t0
    dist = 0.0; jumps = 0; vmax = 0.0
    for a, b in zip(P, P[1:]):
        dd = math.hypot(b[1] - a[1], b[2] - a[2])
        if dd > 0.3: jumps += 1
        else: dist += dd
        vmax = max(vmax, abs(b[4]))
    # 车动时每秒 odom 条数
    moving = [p for p in P if abs(p[4]) > 0.1]
    rate_mv = 0.0
    if len(moving) > 10:
        span = moving[-1][0] - moving[0][0]
        secs = set(int(p[0]) for p in moving)
        rate_mv = len(moving) / max(1, len(secs))
    import time
    st = time.strftime('%H:%M:%S', time.localtime(t0))
    print(f'{name}: 开始 {st}  时长 {dur:6.1f} s  scan {len(sc):5d} ({len(sc)/max(dur,1):4.1f}/s)  odom {len(P):5d}  '
          f'车动时 {rate_mv:5.1f} 条/s  里程 {dist:6.1f} m  最高 {vmax:4.2f} m/s  跳>0.3m {jumps:3d} 次'
          + (f'  终点 ({P[-1][1]:.1f},{P[-1][2]:.1f})' if P else ''))
