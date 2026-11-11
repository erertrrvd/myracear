"""解析 rosbag2 sqlite 里的 /encoder_imu_odom，判断是否有两个来源交替发布。"""
import sqlite3, struct, sys, math, collections

db = sys.argv[1]
con = sqlite3.connect(db)
topics = {r[0]: r[1] for r in con.execute("select id, name from topics")}
tid = [k for k, v in topics.items() if v == '/encoder_imu_odom'][0]
sid = [k for k, v in topics.items() if v == '/scan'][0]


class R:
    def __init__(s, b):
        s.b = b; s.o = 4  # 跳过 CDR 封装头
    def al(s, n):
        s.o += (-(s.o - 4)) % n
    def u32(s):
        s.al(4); v = struct.unpack_from('<I', s.b, s.o)[0]; s.o += 4; return v
    def i32(s):
        s.al(4); v = struct.unpack_from('<i', s.b, s.o)[0]; s.o += 4; return v
    def f64(s, n=1):
        s.al(8); v = struct.unpack_from('<%dd' % n, s.b, s.o); s.o += 8 * n; return v
    def st(s):
        n = s.u32(); v = s.b[s.o:s.o + n - 1].decode('utf8', 'replace'); s.o += n; return v


rows = []
for t, data in con.execute("select timestamp, data from messages where topic_id=? order by timestamp", (tid,)):
    r = R(data)
    sec = r.i32(); nsec = r.u32(); frame = r.st(); child = r.st()
    px, py, pz = r.f64(3); qx, qy, qz, qw = r.f64(4); pcov = r.f64(36)
    vx, vy, vz = r.f64(3); wx, wy, wz = r.f64(3)
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    rows.append((t / 1e9, sec + nsec / 1e9, frame, child, px, py, yaw, vx, wz, pcov[0], pcov[35]))

scans = [t / 1e9 for (t,) in con.execute("select timestamp from messages where topic_id=? order by timestamp", (sid,))]
t0 = rows[0][0]
print('odom 条数', len(rows), ' scan 条数', len(scans), ' 时长 %.1f s' % (rows[-1][0] - t0))
print('frame/child 组合', collections.Counter((r[2], r[3]) for r in rows))
print('协方差组合(前5)', collections.Counter((round(r[9], 6), round(r[10], 6)) for r in rows).most_common(5))

# 每秒条数
per = collections.Counter(int(r[0] - t0) for r in rows)
print('每秒条数分布', sorted(collections.Counter(per.values()).items()))

# 相邻两条的位置跳变；两个来源交替时会出现大量“来回跳”
jumps = []
for a, b in zip(rows, rows[1:]):
    d = math.hypot(b[4] - a[4], b[5] - a[5])
    dy = abs((b[6] - a[6] + math.pi) % (2 * math.pi) - math.pi)
    jumps.append((d, dy, a, b))
big = [j for j in jumps if j[0] > 0.05 or j[1] > math.radians(3)]
print('相邻跳变 >5 cm 或 >3°：', len(big), '次')
for d, dy, a, b in big[:15]:
    print('  t=%.2f  %.3f m  %.1f°  (%.2f,%.2f,%.1f°)->(%.2f,%.2f,%.1f°)  stamp差 %.3f' % (
        a[0] - t0, d, math.degrees(dy), a[4], a[5], math.degrees(a[6]), b[4], b[5], math.degrees(b[6]), b[1] - a[1]))

# 时间戳倒退（两个来源 stamp 交错）
back = sum(1 for a, b in zip(rows, rows[1:]) if b[1] < a[1])
dup = sum(1 for a, b in zip(rows, rows[1:]) if b[1] == a[1])
print('stamp 倒退', back, '次；stamp 相同', dup, '次')

# 试着按“离哪条轨迹近”把消息分成两路
A, B = [], []
for r in rows:
    if not A: A.append(r); continue
    da = math.hypot(r[4] - A[-1][4], r[5] - A[-1][5])
    db_ = math.hypot(r[4] - B[-1][4], r[5] - B[-1][5]) if B else 1e9
    if da <= 0.05 or da <= db_: A.append(r)
    else: B.append(r)
print('按轨迹分路：A %d 条，B %d 条' % (len(A), len(B)))
if B:
    print('  A 终点 (%.2f,%.2f,%.1f°)  B 终点 (%.2f,%.2f,%.1f°)' % (A[-1][4], A[-1][5], math.degrees(A[-1][6]), B[-1][4], B[-1][5], math.degrees(B[-1][6])))

# 车速与里程
dist = sum(j[0] for j in jumps if j[0] < 0.05)
print('总里程(去掉跳变) %.1f m，最大 vx %.2f m/s，最大 |wz| %.2f rad/s' % (dist, max(r[7] for r in rows), max(abs(r[8]) for r in rows)))
print('起点 (%.2f,%.2f,%.1f°) 终点 (%.2f,%.2f,%.1f°)' % (rows[0][4], rows[0][5], math.degrees(rows[0][6]), rows[-1][4], rows[-1][5], math.degrees(rows[-1][6])))
gaps = [b - a for a, b in zip(scans, scans[1:])]
print('scan 最大间隔 %.2f s，>0.5 s 的 %d 次' % (max(gaps), sum(g > 0.5 for g in gaps)))
og = [b[0] - a[0] for a, b in zip(rows, rows[1:])]
print('odom 最大间隔 %.2f s' % max(og))
