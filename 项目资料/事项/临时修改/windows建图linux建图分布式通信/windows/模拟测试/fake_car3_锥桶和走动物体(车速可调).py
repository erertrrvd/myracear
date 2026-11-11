"""模拟小车 3：40m x 4m 房间，12 个细锥桶 + 1 个走动的人，车速可调，且模拟一圈内的运动畸变。
车沿 y=0 以 0.5 m/s 从 x=1 开到 x=11；雷达每帧约 979 点（和实车一致），噪声 1cm，10 Hz。
用法：python fake_car3.py <车速m/s> <时长秒>"""
import math, random, sys, time
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

X0, X1, Y0, Y1 = 0.0, 40.0, -2.0, 2.0
CONES = [(3.0 + 3.0 * i, 1.2 if i % 2 == 0 else -1.2) for i in range(12)]
CONE_R = 0.08
MOVER_X, MOVER_AMP, MOVER_PERIOD, MOVER_R = 20.0, 1.5, 8.0, 0.15
SPEED = float(sys.argv[1]) if len(sys.argv) > 1 else 1.5
START_X = 1.0
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 24.0


def cast(px, py, angles, circles):
    c, s = np.cos(angles), np.sin(angles)
    best = np.full(angles.shape, 25.0)
    with np.errstate(divide='ignore', invalid='ignore'):
        for wall, comp, d in ((Y1, s, py), (Y0, s, py), (X1, c, px), (X0, c, px)):
            t = (wall - d) / comp
            hit = (t > 0.05)
            if comp is s:
                xx = px + t * c
                hit &= (xx >= X0) & (xx <= X1)
            else:
                yy = py + t * s
                hit &= (yy >= Y0) & (yy <= Y1)
            best = np.where(hit & (t < best), t, best)
    for (cx, cy, r) in circles:
        # 射线-圆交点：|p + t*u - c|^2 = r^2
        fx, fy = px - cx, py - cy
        b = fx * c + fy * s
        cc = fx * fx + fy * fy - r * r
        disc = b * b - cc
        ok = disc >= 0
        t = -b - np.sqrt(np.where(ok, disc, 0))
        hit = ok & (t > 0.05)
        best = np.where(hit & (t < best), t, best)
    return best


class Car(Node):
    def __init__(self):
        super().__init__('fake_car2')
        self.ps = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.po = self.create_publisher(Odometry, '/encoder_imu_odom', 10)
        self.t0 = time.time()
        self.create_timer(0.1, self.tick)

    def tick(self):
        el = time.time() - self.t0
        if el > DURATION:
            raise SystemExit
        x = START_X + SPEED * el
        n = random.choice((977, 978, 979))
        inc = 2 * math.pi / n
        angles = -math.pi + inc * np.arange(n)
        my = MOVER_AMP * math.sin(2 * math.pi * el / MOVER_PERIOD)
        circles = [(cx, cy, CONE_R) for cx, cy in CONES] + [(MOVER_X, my, MOVER_R)]
        dts = np.arange(n) / n * 0.1
        r = cast(x + SPEED * dts, 0.0, angles, circles) + np.random.normal(0, 0.01, n)
        now = self.get_clock().now().to_msg()
        sc = LaserScan(); sc.header.stamp = now; sc.header.frame_id = 'laser_link'
        sc.angle_min = float(angles[0]); sc.angle_max = float(angles[-1]); sc.angle_increment = inc
        sc.time_increment = 0.1 / n; sc.scan_time = 0.1; sc.range_min = 0.05; sc.range_max = 25.0
        sc.ranges = [float(v) for v in r]
        self.ps.publish(sc)
        od = Odometry(); od.header.stamp = now; od.header.frame_id = 'odom'; od.child_frame_id = 'base_footprint'
        od.pose.pose.position.x = SPEED * el; od.pose.pose.orientation.w = 1.0
        self.po.publish(od)


rclpy.init()
try:
    rclpy.spin(Car())
except SystemExit:
    pass
