"""离线重建用的播放器：直接读 rosbag2 的 .db3，按原时间间隔发 /scan 和里程计。

里程计话题里如果混着两个来源（两个 imu_encoder_mix 交替发），按位姿连续性分成几条轨迹，
只发选中的一条（--track，默认最长的那条），发到 /key1_odom（建图桥只订它）。
用法：python player.py <bag目录或.db3> [--rate 1.0] [--track N] [--list]
"""
import argparse, glob, math, os, sqlite3, sys, time

ap = argparse.ArgumentParser()
ap.add_argument('bag'); ap.add_argument('--rate', type=float, default=1.0)
ap.add_argument('--track', type=int, default=-1); ap.add_argument('--list', action='store_true')
# 模拟实车 WiFi：里程计（reliable）丢包重传时成批晚到（11-05 17:57 实车晚 0.3~1.8 s），扫描照常到。
# --odom-lag D：每 2 s 里有一段 0.3~D 秒的“卡住”，这段里的里程计攒到卡住结束时一起发；0 = 不模拟
ap.add_argument('--odom-lag', type=float, default=0.0)
args = ap.parse_args()
db = args.bag if args.bag.endswith('.db3') else sorted(glob.glob(os.path.join(args.bag, '*.db3')))[0]
con = sqlite3.connect(db)
topics = {i: (n, t) for i, n, t in con.execute('select id, name, type from topics')}
odom_id = next((i for i, (n, _) in topics.items() if n == '/key1_odom'), None)
if odom_id is None:
    raise RuntimeError('录包没有 /key1_odom，不能用其他里程计替代本次对照')
print('使用录包中的 /key1_odom', flush=True)
scan_id = next(i for i, (n, _) in topics.items() if n == '/scan')

from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# 1. 里程计分轨迹
tracks = []          # 每条：dict(last=(t,x,y), n, ids=set)
assign = {}
if odom_id is not None:
    for rid, t, data in con.execute('select id, timestamp, data from messages where topic_id=? order by timestamp', (odom_id,)):
        m = deserialize_message(data, Odometry)
        t = t / 1e9; x, y = m.pose.pose.position.x, m.pose.pose.position.y
        best = None
        for k, tr in enumerate(tracks):
            lt, lx, ly = tr['last']
            d = math.hypot(x - lx, y - ly)
            if d <= 0.15 + 5.0 * max(0.0, t - lt) and (best is None or d < best[1]):
                best = (k, d)
        if best is None:
            tracks.append(dict(last=(t, x, y), n=0, t0=t, start=(x, y))); k = len(tracks) - 1
        else:
            k = best[0]
        tr = tracks[k]; tr['last'] = (t, x, y); tr['n'] += 1; assign[rid] = k
big = [k for k, tr in enumerate(tracks) if tr['n'] > 50]
print('里程计轨迹：')
for k in big:
    tr = tracks[k]
    print(f'  [{k}] {tr["n"]} 条，{tr["t0"]:.1f}~{tr["last"][0]:.1f} s，起点 ({tr["start"][0]:.2f},{tr["start"][1]:.2f}) 终点 ({tr["last"][1]:.2f},{tr["last"][2]:.2f})')
if args.list:
    sys.exit(0)
pick = args.track if args.track >= 0 else max(big, key=lambda k: tracks[k]['n'])
print(f'只发轨迹 [{pick}]；其他 {len(assign) - tracks[pick]["n"]} 条里程计丢掉', flush=True)

# 2. 播放
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
node = Node('bag_player')
pub_scan = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
pub_odom = node.create_publisher(Odometry, '/key1_odom', 50)
time.sleep(2.0)                                   # 等订阅者发现
rows = con.execute('select id, topic_id, timestamp, data from messages where topic_id in (?, ?) order by timestamp',
                   (scan_id, odom_id if odom_id is not None else -1))
if args.odom_lag > 0:
    import random
    rnd = random.Random(1105)                     # 固定种子：改前 / 改后两次回放卡住的时段一样
    rows = [(rid, tid, t / 1e9, data) for rid, tid, t, data in rows]
    t0, t1 = rows[0][2], rows[-1][2]
    stalls, s = [], t0 + 1.0
    while s < t1:
        stalls.append((s, s + rnd.uniform(0.3, max(0.3, args.odom_lag))))
        s += 2.0
    def release(t):
        for a, b in stalls:
            if a <= t < b:
                return b
        return t
    ev = [(release(t) if tid == odom_id else t, k, rid, tid, t, data) for k, (rid, tid, t, data) in enumerate(rows)]
    ev.sort(key=lambda e: (e[0], e[1]))
    rows = [(rid, tid, rel * 1e9, data) for rel, _, rid, tid, t, data in ev]
    print(f'模拟里程计晚到：{len(stalls)} 段卡住，每段 0.3~{args.odom_lag:.1f} s', flush=True)
start = None; n_scan = n_odom = 0; last_print = time.time()
for rid, tid, t, data in rows:
    t = t / 1e9
    if start is None:
        start = (t, time.time())
    wait = start[1] + (t - start[0]) / args.rate - time.time()
    if wait > 0:
        time.sleep(wait)
    if tid == scan_id:
        pub_scan.publish(deserialize_message(data, LaserScan)); n_scan += 1
    elif assign.get(rid) == pick:
        pub_odom.publish(deserialize_message(data, Odometry)); n_odom += 1
    if time.time() - last_print > 10:
        last_print = time.time()
        print(f'  已播放 {t - start[0]:.0f} s：扫描 {n_scan} 帧、里程计 {n_odom} 条', flush=True)
print(f'播放完：扫描 {n_scan} 帧、里程计 {n_odom} 条', flush=True)
time.sleep(1.0)
node.destroy_node(); rclpy.shutdown()
