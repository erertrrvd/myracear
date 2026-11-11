#!/usr/bin/env python3
"""把 light_infer.py 的识别结果发成 ROS2 话题。

为什么要单独一个进程：推理用的 ais_bench 装在 root 的 conda Python 3.9 里，
ROS2 Humble 的 rclpy 在 /usr/bin/python3（3.10），两者不能放在同一个进程。
所以 light_infer.py 每帧把结果写到状态文件，本节点定时读取并发布。

发布的话题（验证阶段，只发布识别结果，不影响小车运动）：
  /traffic_state      std_msgs/String             red / green / off / none / conflict / stale
  /traffic_light_box  std_msgs/Float32MultiArray  [x1, y1, x2, y2, box_conf, lamp_conf, img_w, img_h, age_s, fps]
                                                  没有灯箱时框为 -1；第二阶段用框高度判断离灯多远

暂时关闭（验证阶段不发，避免干扰运动）：
  /traffic_light      std_msgs/Int32              1 = 绿灯可以走，0 = 不能走
                      pure_track_test 订阅了这个话题（收到 1 会结束红灯等待），接入控制时再打开下面标了 [控制] 的几行

stale：状态文件超过 stale_sec（默认 0.5 秒）没更新，或读不到，说明推理停了或卡了，一律当作不能走。

运行（小车上，必须用 ROS 的 python）：
  source /opt/ros/humble/setup.bash
  /usr/bin/python3 light_bridge.py
  /usr/bin/python3 light_bridge.py --ros-args -p stale_sec:=1.0 -p rate_hz:=30.0
"""
import os
import time

STATES = ("red", "green", "off", "none", "conflict")


def parse_state_line(line):
    """解析 light_infer.write_state 写的一行；格式错误返回 None。
    格式：timestamp state raw x1 y1 x2 y2 box_conf lamp_conf fps W H"""
    p = line.split()
    if len(p) != 12 or p[1] not in STATES:
        return None
    try:
        return dict(ts=float(p[0]), state=p[1], raw=p[2],
                    box=[float(v) for v in p[3:7]], box_conf=float(p[7]), lamp_conf=float(p[8]),
                    fps=float(p[9]), w=float(p[10]), h=float(p[11]))
    except ValueError:
        return None


def read_state(path):
    try:
        with open(path) as f:
            return parse_state_line(f.read().strip())
    except OSError:
        return None


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray, String
    # from std_msgs.msg import Int32                                              # [控制] 验证阶段关闭

    class LightBridge(Node):
        def __init__(self):
            super().__init__("light_bridge")
            self.declare_parameter("state_file", "/tmp/light_state.txt")
            self.declare_parameter("stale_sec", 0.5)
            self.declare_parameter("rate_hz", 20.0)
            self.path = self.get_parameter("state_file").value
            self.stale = float(self.get_parameter("stale_sec").value)
            rate = float(self.get_parameter("rate_hz").value)
            self.pub_state = self.create_publisher(String, "traffic_state", 10)
            # self.pub_go = self.create_publisher(Int32, "traffic_light", 10)      # [控制] 验证阶段关闭，避免影响 pure_track_test
            self.pub_box = self.create_publisher(Float32MultiArray, "traffic_light_box", 10)
            self.last = None
            self.create_timer(1.0 / rate, self.tick)
            self.get_logger().info(f"读取 {self.path}，{rate:.0f} Hz 发布 /traffic_state /traffic_light_box（验证模式，不发 /traffic_light）")

        def tick(self):
            d = read_state(self.path)
            age = time.time() - d["ts"] if d else -1.0
            if d is None or age > self.stale or age < -1.0:   # age<-1：时钟异常，也当作无效
                state = "stale"
            else:
                state = d["state"]
            go = 1 if state == "green" else 0

            self.pub_state.publish(String(data=state))
            # self.pub_go.publish(Int32(data=go))                                 # [控制] 验证阶段关闭
            if d:
                self.pub_box.publish(Float32MultiArray(data=d["box"] + [d["box_conf"], d["lamp_conf"],
                                                                        d["w"], d["h"], float(age), d["fps"]]))
            if state != self.last:
                why = "" if state != "stale" else (f"（{age:.1f} 秒没更新）" if d else "（读不到状态文件）")
                self.get_logger().info(f"状态 {self.last} -> {state}{why}  {'GO' if go else 'STOP'}（仅显示，未发布）")
                self.last = state

    rclpy.init()
    node = LightBridge()
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
