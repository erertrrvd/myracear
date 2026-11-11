"""模拟小车：10m x 3m 走廊，车沿 x 方向 0.25 m/s 直行，发 /scan(366~368 点随机) 和 /encoder_imu_odom。
时间戳用本机时钟（电脑上自测，不测时钟偏差）。"""
import math, random, sys, time
NOODOM = len(sys.argv) > 2 and sys.argv[2] == 'noodom'
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

L, W = 10.0, 3.0           # 走廊 x∈[0,L], y∈[-W/2,W/2]
SPEED, RATE, DURATION = 0.25, 10.0, float(sys.argv[1]) if len(sys.argv) > 1 else 24.0


def raycast(px, angles):
    out = []
    for a in angles:
        c, s = math.cos(a), math.sin(a)
        best = 25.0
        # y = ±W/2 墙
        if abs(s) > 1e-9:
            for wy in (W / 2, -W / 2):
                t = (wy - 0.0) / s
                if t > 0.05 and 0 <= px + t * c <= L: best = min(best, t)
        # x = 0 / L 端墙
        if abs(c) > 1e-9:
            for wx in (0.0, L):
                t = (wx - px) / c
                if t > 0.05 and abs(t * s) <= W / 2: best = min(best, t)
        out.append(best)
    return out


class Car(Node):
    def __init__(self):
        super().__init__('fake_car')
        self.ps = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.po = self.create_publisher(Odometry, '/encoder_imu_odom', 10)
        self.t0 = time.time()
        self.create_timer(1.0 / RATE, self.tick)

    def tick(self):
        el = time.time() - self.t0
        if el > DURATION:
            self.get_logger().info('done'); raise SystemExit
        x = 1.0 + SPEED * el
        n = random.choice((366, 367, 368))
        inc = 2 * math.pi / n
        angles = [-math.pi + i * inc for i in range(n)]
        now = self.get_clock().now().to_msg()
        sc = LaserScan(); sc.header.stamp = now; sc.header.frame_id = 'laser_link'
        sc.angle_min = angles[0]; sc.angle_max = angles[-1]; sc.angle_increment = inc
        sc.time_increment = 0.1 / n; sc.scan_time = 0.1; sc.range_min = 0.05; sc.range_max = 25.0
        sc.ranges = [float(r) for r in raycast(x, angles)]
        self.ps.publish(sc)
        od = Odometry(); od.header.stamp = now; od.header.frame_id = 'odom'; od.child_frame_id = 'base_footprint'
        od.pose.pose.position.x = SPEED * el; od.pose.pose.orientation.w = 1.0
        if not NOODOM: self.po.publish(od)


rclpy.init()
try:
    rclpy.spin(Car())
except SystemExit:
    pass
