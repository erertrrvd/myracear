#!/usr/bin/env python3
"""离线回放中按扫描时间记录 SLAM 与里程计位姿；不发布任何话题。"""
import csv
import math
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class Recorder(Node):
    def __init__(self, path):
        super().__init__('offline_slam_pose_recorder')
        self.output = open(path, 'w', encoding='utf-8', newline='')
        self.writer = csv.writer(self.output)
        self.writer.writerow(['stamp_sec', 'map_x', 'map_y', 'map_yaw', 'odom_x', 'odom_y', 'odom_yaw'])
        self.tf = Buffer(cache_time=Duration(seconds=20.0))
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(LaserScan, '/scan_pc', self.on_scan, qos_profile_sensor_data)
        self.count = 0

    def on_scan(self, scan):
        when = Time.from_msg(scan.header.stamp)
        try:
            poses = [self.tf.lookup_transform(frame, 'base_pc', when, timeout=Duration(seconds=.025)).transform
                     for frame in ('map', 'odom_pc')]
        except Exception:
            return
        row = [when.nanoseconds / 1e9]
        for p in poses:
            q = p.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            row.extend([p.translation.x, p.translation.y, yaw])
        self.writer.writerow(row)
        self.count += 1
        if self.count % 20 == 0:
            self.output.flush()


def main():
    path = sys.argv[1]
    rclpy.init(args=[])
    node = Recorder(path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.output.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
