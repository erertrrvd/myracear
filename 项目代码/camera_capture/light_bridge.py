#!/usr/bin/env python3
"""红绿灯话题节点：把控制节点的请求转成"开/关识别"的命令，发给守护进程。

为什么不是"用完就杀进程"：
    这块板子上 yolo 的启动成本很高（import torch/cv2/ais_bench 约 11 秒，
    加加载 .om、初始化 NPU、开摄像头，冷启动 14~15 秒）。竞速比赛不可能在
    红绿灯前等这么久。所以改成：

        start_vision.sh 先把守护进程起来（发车前，摆车的时间就把模型加载好）
                      ↓
        跑车全程：守护进程 socket 待命，不读摄像头、不推理，CPU ≈ 0
                      ↓
        到停车点：pure_track_test 发 /traffic_light_req = 1
                  → 本节点给守护进程发 "on" → 立刻开始认灯（约 100ms 出结果）
                      ↓
        认出绿灯：本节点发 /traffic_light = 1，同时给守护进程发 "off"
                  → 关摄像头、停推理，回到待命（CPU 回 0）

话题：
  /traffic_light_req  std_msgs/Int32   订阅  1 = 车已停在红绿灯点，开始认灯
                                             0 = 车要走了，把识别关掉
  /traffic_light      std_msgs/Int32   发布  1 = 确认绿灯，可以走
  /traffic_state      std_msgs/String  发布  red / green / off / none / conflict / stale
  /traffic_light_box  std_msgs/Float32MultiArray  发布
                                             [x1,y1,x2,y2,灯箱置信度,亮灯置信度,图宽,图高,结果延迟秒,推理fps]

守护进程不在时怎么办：本节点照常订阅和定时，只是发不出绿灯（日志会提示），
由 pure_track_test 那边的超时兜底放行。不会卡死。

运行（小车上，必须用 ROS 的 python）：
  source /opt/ros/humble/setup.bash
  cd /home/racecar/src/my_racecar/camera_capture
  /usr/bin/python3 light_bridge.py
"""
import argparse
import os
import socket
import sys
import threading
import time

# 识别可能输出的状态；stale 是本节点自己加的（状态文件超时）
STATES = ("red", "green", "off", "none", "conflict")

DEFAULT_SOCKET = "/tmp/light_infer.sock"
DEFAULT_MODEL = "output/yolov5n_bs1.om"


# ============================================================ 纯逻辑（不依赖 rclpy，可在电脑上单测）
def parse_state_line(line):
    """解析 light_infer 系列写的一行；格式错误返回 None。

    格式：ts state raw x1 y1 x2 y2 box_conf lamp_conf fps W H
    """
    p = line.split()
    if len(p) != 12 or p[1] not in STATES:
        return None
    try:
        return dict(ts=float(p[0]), state=p[1], raw=p[2],
                    box=[float(v) for v in p[3:7]],
                    box_conf=float(p[7]), lamp_conf=float(p[8]),
                    fps=float(p[9]), w=float(p[10]), h=float(p[11]))
    except ValueError:
        return None


def read_state(path):
    """读状态文件；不存在 / 读不了 / 格式错都返回 None。"""
    try:
        with open(path) as f:
            return parse_state_line(f.read().strip())
    except OSError:
        return None


def judge(d, now, stale_sec):
    """把状态文件内容判成 (状态, 延迟秒)。

    d 为 None（读不到）、时间戳明显来自未来、或超过 stale_sec 没更新，一律给 "stale"。
    """
    if d is None:
        return "stale", -1.0
    age = now - d["ts"]
    if age > stale_sec or age < -1.0:      # age<-1：两边时钟不一致，也当无效
        return "stale", age
    return d["state"], age


def tick_decide(monitoring, was_go, d, now, stale_sec):
    """一个周期的判定（纯函数，方便用模拟时钟测）。

    返回 (下一周期是否继续监控, 本周期要不要发 /traffic_light=1, 本周期状态)

    规则：
      - 没在监控 -> 不发
      - 在监控但状态不是新鲜的 green -> 不发（红灯/没灯/识别挂了都继续等）
      - 认出 green -> 发一次，并且停止监控（调用方负责给守护进程发 off）
      - 发过之后不再重复发，直到控制节点重新请求（req=1 会把 was_go 清掉）
    """
    if not monitoring:
        return False, False, "off"
    state, age = judge(d, now, stale_sec)
    if state == "green" and not was_go:
        return False, True, "green"
    return True, False, state


class InferClient:
    """守护进程的客户端。

    只管发命令，不管进程死活 —— 连不上就返回 False，由调用方决定怎么办。
    所有方法都不抛异常，不能因为 socket 出问题把 ROS 节点带崩。
    """

    def __init__(self, addr, timeout=3.0):
        self.addr = addr
        self.timeout = timeout

    def _connect(self):
        if ":" in self.addr:
            host, port = self.addr.rsplit(":", 1)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self.timeout)
            s.connect((host, int(port)))
        else:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.settimeout(self.timeout)
            s.connect(self.addr)
        return s

    def command(self, cmd, expect_reply=False):
        """发一条命令。返回 (是否成功, 回复文本)。"""
        try:
            s = self._connect()
        except OSError as e:
            return False, f"连不上守护进程 {self.addr}：{e}"
        try:
            s.sendall((cmd + "\n").encode())
            if not expect_reply:
                return True, ""
            buf = b""
            while b"\n" not in buf:
                chunk = s.recv(256)
                if not chunk:
                    break
                buf += chunk
            return True, buf.decode(errors="replace").strip()
        except OSError as e:
            return False, f"发命令失败：{e}"
        finally:
            try:
                s.close()
            except OSError:
                pass

    def ping(self):
        ok, reply = self.command("ping", expect_reply=True)
        return ok and reply == "pong"


class InferHeartbeat:
    """监控期间每隔 interval 秒给守护进程发一句 "stop" 当心跳。

    为什么需要：bridge 每发一条命令就新建连接、发完就关（简单可靠），
    守护进程因此不能拿"连接断开"当"bridge 走了"。它改成看【多久没收到命令】，
    所以监控期间必须有人定期说话，否则 3 秒后它就以为 bridge 挂了、把摄像头关了。

    心跳是后台线程，不受 ROS 回调阻塞影响。
    """

    def __init__(self, client, interval=1.0):
        self.client = client
        self.interval = interval
        self._thread = None
        self._stop = threading.Event()

    def start(self):
        """开始发心跳（幂等）。"""
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        """停止发心跳（幂等）。"""
        self._stop.set()
        t, self._thread = self._thread, None
        if t is not None:
            t.join(timeout=2.0)

    def _loop(self):
        while not self._stop.wait(self.interval):
            if not self.client.command("stop"):
                # 守护进程不在了，停掉心跳，免得空转
                self._stop.set()
                return


def abspath_from_here(path, here):
    """把相对路径按「相对 camera_capture 目录」解释。"""
    return path if os.path.isabs(path) else os.path.normpath(os.path.join(here, path))


# ============================================================ ROS 节点
def main():
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--socket", default=DEFAULT_SOCKET, help="守护进程的 socket")
    ap.add_argument("--state-file", default="/tmp/light_state.txt")
    known, _ = ap.parse_known_args()          # 本脚本的参数先摘掉

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray, Int32, String

    here = os.path.dirname(os.path.abspath(__file__))
    client = InferClient(known.socket)
    if not client.ping():
        print(f"警告：连不上识别守护进程 {known.socket}。\n"
              f"      先运行 bash start_vision.sh 把守护进程起来，否则认不出绿灯\n"
              f"      （pure_track_test 那边有超时兜底，不会把车卡死）", flush=True)

    class LightBridge(Node):
        def __init__(self):
            super().__init__("light_bridge")
            self.declare_parameter("state_file", known.state_file)
            self.declare_parameter("stale_sec", 0.5)
            self.declare_parameter("rate_hz", 20.0)
            self.path = self.get_parameter("state_file").value
            self.stale = float(self.get_parameter("stale_sec").value)
            rate = float(self.get_parameter("rate_hz").value)

            self.monitoring = False        # 控制节点有没有要求认灯
            self.was_go = False
            self.last_state = None
            self.warned = False            # 守护进程连不上只警告一次
            self.hb = InferHeartbeat(client, interval=1.0)

            self.pub_state = self.create_publisher(String, "traffic_state", 10)
            self.pub_box = self.create_publisher(Float32MultiArray, "traffic_light_box", 10)
            self.pub_go = self.create_publisher(Int32, "traffic_light", 10)
            self.create_subscription(Int32, "traffic_light_req", self.on_req, 10)
            self.create_timer(1.0 / rate, self.tick)

            self.get_logger().info("等 /traffic_light_req：1 = 开始认灯，0 = 关掉识别")
            self.get_logger().info(f"状态文件 {self.path}，{rate:.0f} Hz")

        # ---------------------------------------------- 控制节点的请求
        def on_req(self, msg):
            if msg.data == 1:
                if self.monitoring:
                    return
                self.monitoring = True
                self.was_go = False
                ok, why = client.command("on")
                if ok:
                    self.warned = False
                    self.hb.start()            # 开始发心跳，别让守护进程以为我们挂了
                    self.get_logger().info("收到请求：开始认灯（守护进程已开始推理）")
                else:
                    self.get_logger().error(f"收到请求，但发不出 on：{why}")
            else:
                if not self.monitoring:
                    return
                self.monitoring = False
                self.was_go = False
                self.hb.stop()                 # 先停心跳，再关识别
                ok, why = client.command("off")
                if ok:
                    self.get_logger().info("收到请求：关掉识别（摄像头已释放，CPU 回到 0）")
                else:
                    self.get_logger().warn(f"发不出 off：{why}")

        # ---------------------------------------------- 定时：读状态、发话题
        def tick(self):
            now = time.time()
            d = read_state(self.path) if self.monitoring else None
            state, age = judge(d, now, self.stale)

            # 监控期间把状态和灯箱框发出去（控制节点只看 /traffic_light，这两个是调试用）
            if self.monitoring:
                self.pub_state.publish(String(data=state))
                if d is not None:
                    self.pub_box.publish(Float32MultiArray(
                        data=d["box"] + [d["box_conf"], d["lamp_conf"], d["w"], d["h"],
                                         float(age), d["fps"]]))

            nxt, go, _ = tick_decide(self.monitoring, self.was_go, d, now, self.stale)
            if go:
                self.get_logger().info("认出绿灯（延迟 %.0fms），关掉识别并通知控制节点放行"
                                       % (age * 1000))
                self.hb.stop()                  # 先停心跳，再关识别
                client.command("off")           # 先关识别再放行：车一动 CPU 就是干净的
                self.pub_go.publish(Int32(data=1))
                self.was_go = True
            self.monitoring = nxt

            if self.monitoring and state != self.last_state:
                why = "" if d is not None else "（读不到状态文件）"
                self.get_logger().info(f"状态 {self.last_state} -> {state}{why}")
            if self.monitoring:
                self.last_state = state

        def destroy_node(self):
            # 节点退出时顺手把心跳和识别都停掉，别让摄像头一直被占着
            try:
                self.hb.stop()
                client.command("off")
            except Exception:
                pass
            super().destroy_node()

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
