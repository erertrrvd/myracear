#!/usr/bin/env python3
"""接收端示例：订阅 light_bridge.py 发布的红绿灯话题并打印，用来确认话题收得到。
以后控制节点（如 pure_track_test）就按这里的方式订阅。

运行（小车上，或同一 ROS_DOMAIN_ID 的电脑上）：
  source /opt/ros/humble/setup.bash
  /usr/bin/python3 light_listener.py
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
# from std_msgs.msg import Int32                                                   # [控制] 验证阶段关闭


class LightListener(Node):
    def __init__(self):
        super().__init__("light_listener")
        self.state, self.go, self.box = None, None, None
        self.n, self.t0 = 0, time.time()
        self.create_subscription(String, "traffic_state", self.on_state, 10)
        # self.create_subscription(Int32, "traffic_light", self.on_go, 10)       # [控制] 验证阶段 bridge 不发这个话题
        self.create_subscription(Float32MultiArray, "traffic_light_box", self.on_box, 10)
        self.create_timer(2.0, self.report)
        self.get_logger().info("等待 /traffic_state /traffic_light_box ...")

    def on_state(self, msg):
        self.n += 1
        if msg.data != self.state:
            self.get_logger().info(f"状态变化：{self.state} -> {msg.data}")
            self.state = msg.data

    def on_go(self, msg):
        self.go = msg.data

    def on_box(self, msg):
        self.box = list(msg.data)

    def report(self):
        hz = self.n / max(time.time() - self.t0, 1e-6)
        self.n, self.t0 = 0, time.time()
        if self.box and self.box[0] >= 0:
            x1, y1, x2, y2, bc, lc, w, h, age, fps = self.box
            box = f"灯箱 ({x1:.0f},{y1:.0f})-({x2:.0f},{y2:.0f}) 高{y2 - y1:.0f}px 置信度{bc:.2f} 亮灯{lc:.2f} 延迟{age * 1000:.0f}ms 推理{fps:.1f}fps"
        else:
            box = "没有灯箱"
        # go = {1: "GO", 0: "STOP"}.get(self.go, "?")                          # [控制] 来自 /traffic_light
        go = "GO" if self.state == "green" else "STOP"                            # 验证阶段：由状态推算，仅显示
        self.get_logger().info(f"[{hz:.0f}Hz] 状态={self.state} {go} | {box}")


def main():
    rclpy.init()
    node = LightListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
