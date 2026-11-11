"""EKF 冒烟：假装小车（时钟快 1 年）发 /encoder（3 m/s，50 Hz）、/IMU_data（0.5 rad/s，100 Hz）、/key1_odom；
收 /ekf_odom，检查：有输出、时间戳是车上时间（不换算）、轨迹是半径 6 m 的圆。跑 12 s。"""
import math, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.time import Time

YEAR = 365 * 86400.0
V, W = 3.0, 0.5
rclpy.init(); n = Node('fake_car')
pe = n.create_publisher(Odometry, '/encoder', 50)
pi = n.create_publisher(Imu, '/IMU_data', 100)
pk = n.create_publisher(Odometry, '/key1_odom', 50)
got = []
n.create_subscription(Odometry, '/ekf_odom', lambda m: got.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                      m.pose.pose.position.x, m.pose.pose.position.y, time.time())), 50)
def st(t):
    s = math.floor(t); return Time(seconds=s, nanoseconds=int((t - s) * 1e9)).to_msg()
t0 = time.time(); k = 0
while time.time() - t0 < 12:
    now = time.time(); car = now + YEAR; el = now - t0
    if k % 2 == 0:
        e = Odometry(); e.header.stamp = st(car); e.twist.twist.linear.x = V; pe.publish(e)
        o = Odometry(); o.header.stamp = st(car); yaw = W * el
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2); pk.publish(o)
    m = Imu(); m.header.stamp = st(car); m.angular_velocity.z = W; pi.publish(m)
    k += 1
    rclpy.spin_once(n, timeout_sec=0.0)
    time.sleep(max(0.0, t0 + k * 0.01 - time.time()))
print('EKF 输出', len(got), '条')
if got:
    s, x, y, r = got[-1]
    print('最后一条：时间戳-电脑时间 = %.1f 天（应约 365）' % ((s - r) / 86400))
    R = V / W
    errs = [abs(math.hypot(x, y - R) - R) for _, x, y, _ in got[len(got) // 3:]]
    print('位置 (%.2f, %.2f)；到圆心(0,%.0f) 的距离与半径 %.0f m 之差：最大 %.2f m' % (x, y, R, R, max(errs)))
    print('走过弧长约 %.1f m（应约 %.1f m）' % (sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(got, got[1:])), V * (got[-1][3] - got[0][3])))
