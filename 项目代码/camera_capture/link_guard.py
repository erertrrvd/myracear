#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PC 心跳看门狗（跑在小车上）

作用：
    电脑每 200ms 发一个 UDP 心跳过来；这里 0.5 秒收不到，就【停发】/pc_heartbeat。
    transform_v 那边 0.25 秒收不到 /pc_heartbeat 就刹车。
    两段串起来：电脑断联后最慢 0.75 秒车停住。

为什么用 UDP 而不是 ROS 话题：
    这个话题必须真的穿过 Wi-Fi。跑在同一条链路上的 UDP 才是真链路检测，
    跑在车上的 ros2 topic pub 只是"SSH 会话还在不在"的替身，不准也不快。

启动（注意用 /usr/bin/python3，conda 的 python 没有 rclpy）：
    source /opt/ros/humble/setup.bash
    /usr/bin/python3 link_guard.py
"""
import socket
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class LinkGuard(Node):
    def __init__(self):
        super().__init__("link_guard")

        self.declare_parameter("port", 9999)
        self.declare_parameter("udp_timeout", 0.5)   # 收不到心跳多久判定断线
        self.declare_parameter("bind_addr", "0.0.0.0")  # 0.0.0.0 = 所有网卡都听

        port = int(self.get_parameter("port").value)
        self.udp_timeout = float(self.get_parameter("udp_timeout").value)
        bind_addr = str(self.get_parameter("bind_addr").value)

        # ---- UDP socket（非阻塞，不开线程）----
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.bind((bind_addr, port))
        except OSError as e:
            self.get_logger().fatal(f"绑定 UDP {bind_addr}:{port} 失败：{e}")
            raise
        self.sock.setblocking(False)

        self.pub = self.create_publisher(Empty, "pc_heartbeat", 10)

        self.last_rx = None          # 用 monotonic，不怕系统时间被改
        self.link_ok = False
        self.peer = None
        self.rx_count = 0

        self.create_timer(0.05, self.tick)   # 20 Hz

        self.get_logger().info(
            f"link_guard 启动：监听 UDP {bind_addr}:{port}，"
            f"{self.udp_timeout}s 没心跳就停发 /pc_heartbeat")

    def tick(self):
        now = time.monotonic()

        # 把 socket 缓冲里的包一次读干净（可能一次来好几个）
        got = False
        while True:
            try:
                _, addr = self.sock.recvfrom(64)
                got = True
                self.peer = addr[0]
            except BlockingIOError:
                break
            except OSError:
                break

        if got:
            self.last_rx = now
            self.rx_count += 1
            if not self.link_ok:
                self.link_ok = True
                self.get_logger().info(f"PC 心跳建立（来自 {self.peer}）")

        ok = (self.last_rx is not None) and (now - self.last_rx < self.udp_timeout)

        if self.link_ok and not ok:
            self.link_ok = False
            age = (now - self.last_rx) * 1000.0 if self.last_rx else -1.0
            self.get_logger().error(
                f"!! PC 心跳丢失（最后一个在 {age:.0f} ms 前，共收到 {self.rx_count} 个）"
                f" -> 停发 /pc_heartbeat，车会刹车 !!")

        # 只有链路好才发；链路一断就沉默，让 transform_v 靠超时判断
        if ok:
            self.pub.publish(Empty())


def main():
    rclpy.init()
    node = LinkGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
