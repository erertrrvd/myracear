#!/usr/bin/env python3
"""小车端：摄像头采集 → 发给电脑识别 → 收回结果 → 发布 ROS 2 话题。

小车只做采集、JPEG 编码、收发数据，**不用 conda、CANN、torch、NPU**，识别在电脑上完成。
发布的话题与 light_bridge.py 完全相同（下游不用改）：
  /traffic_state      std_msgs/String             red / green / off / none / conflict / stale
  /traffic_light_box  std_msgs/Float32MultiArray  [x1,y1,x2,y2,box_conf,lamp_conf,W,H,age_s,fps]
  （/traffic_light 验证阶段不发，避免影响 pure_track_test）
stale：没有电脑连接，或最新结果距拍摄时刻超过 stale_sec（默认 0.5 秒）。只有 green 才算放行。
延迟全部用小车自己的单调时钟计算，不拿电脑时间和小车时间比（小车时钟本身不准）。

运行（小车上，必须用 ROS 的 python）：
  /usr/bin/python3 remote_vision_bridge.py --bind 0.0.0.0 --port 9000
  /usr/bin/python3 remote_vision_bridge.py --selftest        # 只测采集和编码的负载，不连电脑
  /usr/bin/python3 remote_vision_bridge.py --no-ros --source 某视频.mp4 --bind 127.0.0.1   # 电脑上联调
"""
import argparse
import collections
import os
import socket
import sys
import threading
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import remote_proto as rp

mono = time.monotonic


class Latest:
    """线程安全的“最新一帧”：新帧覆盖旧帧，发送线程永远只拿最新的（丢旧帧，不排队）"""

    def __init__(self):
        self.cv = threading.Condition()
        self.fid, self.t, self.jpeg = 0, 0.0, None
        self.times = collections.OrderedDict()      # 帧号 → 拍摄时刻（小车单调时钟），用来算延迟

    def put(self, jpeg, t):
        with self.cv:
            self.fid += 1
            self.t, self.jpeg = t, jpeg
            self.times[self.fid] = t
            while len(self.times) > 400:
                self.times.popitem(last=False)
            self.cv.notify_all()

    def get_newer(self, last, timeout):
        with self.cv:
            self.cv.wait_for(lambda: self.fid > last, timeout)
            return (self.fid, self.t, self.jpeg) if self.fid > last else None


class ResultState:
    """电脑回传结果的当前状态；超时或断线按 stale 处理（纯逻辑，便于测试）"""

    def __init__(self, stale_sec):
        self.lock = threading.Lock()
        self.stale_sec = stale_sec
        self.res, self.t_cap, self.connected = None, 0.0, False
        self.arrivals = collections.deque(maxlen=30)

    def set_connected(self, v):
        with self.lock:
            self.connected = v
            if not v:
                self.res = None

    def update(self, res, t_cap):
        with self.lock:
            if self.res is None or t_cap >= self.t_cap:      # 乱序到达的旧结果不覆盖新结果
                self.res, self.t_cap = res, t_cap
            self.arrivals.append(mono())

    def snapshot(self, now=None):
        now = mono() if now is None else now
        with self.lock:
            res, t_cap, conn = self.res, self.t_cap, self.connected
            arr = list(self.arrivals)
        fps = 0.0
        if len(arr) >= 2 and now - arr[0] > 0:
            recent = [x for x in arr if now - x <= 2.0]
            fps = (len(recent) - 1) / max(recent[-1] - recent[0], 1e-6) if len(recent) >= 2 else 0.0
        if res is None or not conn:
            return dict(state="stale", box=None, lamp=0.0, w=0, h=0, age=-1.0, fps=0.0, conn=conn)
        age = now - t_cap
        state = res["state"] if age <= self.stale_sec else "stale"
        return dict(state=state, box=res.get("box"), lamp=float(res.get("lamp_conf", 0.0)),
                    w=int(res.get("w", 0)), h=int(res.get("h", 0)), age=age, fps=fps, conn=conn)


class Stats:
    def __init__(self):
        self.lock = threading.Lock()
        self.cap = self.sent = self.res = self.bytes = 0
        self.enc_ms, self.rtt = [], []
        self.t0 = mono()

    def report(self):
        with self.lock:
            dt = max(mono() - self.t0, 1e-6)
            rtt = sorted(self.rtt)
            p = lambda q: (rtt[min(int(len(rtt) * q), len(rtt) - 1)] * 1000) if rtt else -1
            enc = sum(self.enc_ms) / len(self.enc_ms) if self.enc_ms else 0
            line = (f"采集 {self.cap / dt:.1f} fps，编码 {enc:.1f} ms，发送 {self.sent / dt:.1f} fps "
                    f"{self.bytes / dt / 1024:.0f} KB/s，收到结果 {self.res / dt:.1f}/s，"
                    f"往返延迟 p50 {p(0.5):.0f} ms p95 {p(0.95):.0f} ms")
            self.cap = self.sent = self.res = self.bytes = 0
            self.enc_ms, self.rtt, self.t0 = [], [], mono()
        return line


def open_source(a):
    src = int(a.source) if a.source.isdigit() else a.source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        return None
    if isinstance(src, int):                          # 摄像头：MJPG 能省 USB 带宽和解码开销
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def capture_loop(a, latest, stats, stop):
    interval, cap = 1.0 / a.fps, None
    while not stop.is_set():
        if cap is None:
            cap = open_source(a)
            if cap is None:
                print(f"打不开 {a.source}（可能被别的程序占用），2 秒后重试")
                time.sleep(2)
                continue
        t_start = mono()
        ok, frame = cap.read()
        if not ok:
            if not a.source.isdigit():                # 视频文件：播完从头再来
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            cap.release()
            cap = None
            continue
        t_cap = mono()
        if frame.shape[1] != a.width or frame.shape[0] != a.height:
            frame = cv2.resize(frame, (a.width, a.height))
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, a.quality])
        if not ok:
            continue
        latest.put(buf.tobytes(), t_cap)
        with stats.lock:
            stats.cap += 1
            stats.enc_ms.append((mono() - t_cap) * 1000)
        rest = interval - (mono() - t_start)
        if rest > 0:
            time.sleep(rest)
    if cap is not None:
        cap.release()


def recv_loop(conn, ep, latest, res, stats, stop, cur):
    rd = rp.LineReader(conn)
    try:
        while not stop.is_set() and cur["ep"] == ep:
            d = rd.read()
            if not rp.valid_result(d):
                continue
            t = latest.times.get(d["id"])
            if t is None:                              # 太旧的帧号，已经查不到拍摄时刻
                continue
            with stats.lock:
                stats.res += 1
                stats.rtt.append(mono() - t)
            res.update(d, t)
    except (OSError, ValueError):                      # 含 ConnectionError、JSON 解析错误
        pass
    finally:
        try:
            conn.close()                               # 让发送线程也跟着退出
        except OSError:
            pass


def serve_client(a, conn, addr, ep, latest, res, stats, stop, cur):
    rp.enable_keepalive(conn)
    res.set_connected(True)
    threading.Thread(target=recv_loop, args=(conn, ep, latest, res, stats, stop, cur), daemon=True).start()
    last = 0
    try:
        while not stop.is_set() and cur["ep"] == ep:
            item = latest.get_newer(last, 0.5)
            if item is None:
                continue
            fid, t, jpeg = item
            rp.send_frame(conn, fid, t, jpeg)
            last = fid
            with stats.lock:
                stats.sent += 1
                stats.bytes += len(jpeg)
    except OSError as e:
        print("发送中断：", e)
    finally:
        try:
            conn.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        conn.close()
        if cur["ep"] == ep:
            res.set_connected(False)
            print(f"电脑 {addr[0]} 已断开，状态进入 stale")


def server_loop(a, latest, res, stats, stop, cur):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((a.bind, a.port))
    srv.listen(2)
    srv.settimeout(1.0)
    print(f"等待电脑连接 {a.bind}:{a.port}（新连接会顶掉旧连接）")
    while not stop.is_set():
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            continue
        old = cur["conn"]
        cur["ep"] += 1
        cur["conn"] = conn
        if old is not None:
            try:
                old.close()
            except OSError:
                pass
        print(f"电脑已连接 {addr[0]}:{addr[1]}")
        threading.Thread(target=serve_client, args=(a, conn, addr, cur["ep"], latest, res, stats, stop, cur),
                         daemon=True).start()


def selftest(a, latest, stats):
    """只测采集和编码：帧率、编码耗时、JPEG 大小、本进程 CPU 占用（占一个核的百分比）"""
    secs = a.selftest_seconds
    print(f"自测 {secs} 秒：只采集和 JPEG 编码，不连电脑")
    c0, t0, sizes, last = time.process_time(), mono(), [], 0
    while mono() - t0 < secs:
        it = latest.get_newer(last, 1.0)
        if it:
            last = it[0]
            sizes.append(len(it[2]))
    wall, cpu = mono() - t0, time.process_time() - c0
    with stats.lock:
        enc = sum(stats.enc_ms) / len(stats.enc_ms) if stats.enc_ms else 0
        n = stats.cap
    kb = sum(sizes) / len(sizes) / 1024 if sizes else 0
    print(f"采集 {n / wall:.1f} fps，编码 {enc:.1f} ms/帧，JPEG {kb:.0f} KB/帧，"
          f"按此帧率约 {kb * n / wall:.0f} KB/s；本进程 CPU {cpu / wall * 100:.0f}%（100% = 占满一个核）")


def console_loop(a, res, stats, stop):
    last, t_rep = None, mono()
    try:
        while True:
            s = res.snapshot()
            if s["state"] != last:
                print(f"[{time.strftime('%H:%M:%S')}] 状态 {last} -> {s['state']}  "
                      f"延迟 {s['age'] * 1000:.0f} ms  {'GO' if s['state'] == 'green' else 'STOP'}")
                last = s["state"]
            if mono() - t_rep >= 5:
                print(stats.report())
                t_rep = mono()
            time.sleep(0.05)
    except KeyboardInterrupt:
        stop.set()


def ros_loop(a, res, stats, stop):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray, String
    from std_msgs.msg import Int32

    class Bridge(Node):
        def __init__(self):
            super().__init__("remote_vision_bridge")
            self.pub_state = self.create_publisher(String, "traffic_state", 10)
            self.pub_box = self.create_publisher(Float32MultiArray, "traffic_light_box", 10)
            # 控制接入（--enable-control）：只在 pure_track_test 发出 /traffic_light_req=1 期间，才发布 /traffic_light
            self.control, self.req = a.enable_control, False
            if self.control:
                self.pub_go = self.create_publisher(Int32, "traffic_light", 10)
                self.create_subscription(Int32, "traffic_light_req", self.on_req, 10)
            self.last, self.t_rep = None, mono()
            self.create_timer(1.0 / a.rate, self.tick)
            self.get_logger().info(f"发布 /traffic_state /traffic_light_box（{a.rate:.0f} Hz，{'控制模式：收到 /traffic_light_req=1 期间发 /traffic_light' if self.control else '验证模式，不发 /traffic_light'}）")

        def on_req(self, msg):
            want = msg.data == 1
            if want != self.req:
                self.req = want
                self.get_logger().info(f"收到 /traffic_light_req={msg.data}：{'开始' if want else '停止'}发布 /traffic_light")

        def tick(self):
            s = res.snapshot()
            self.pub_state.publish(String(data=s["state"]))
            if self.control and self.req:   # 只有明确 green 才发 1；stale/off/none/red/conflict 都发 0
                self.pub_go.publish(Int32(data=1 if s["state"] == "green" else 0))
            if s["age"] >= 0:
                b = [float(v) for v in (s["box"] or [-1, -1, -1, -1, 0])]
                self.pub_box.publish(Float32MultiArray(data=b + [s["lamp"], float(s["w"]), float(s["h"]),
                                                                 float(s["age"]), float(s["fps"])]))
            if s["state"] != self.last:
                self.get_logger().info(f"状态 {self.last} -> {s['state']}  延迟 {s['age'] * 1000:.0f} ms  "
                                       f"{'GO' if s['state'] == 'green' else 'STOP'}{'' if self.control else '（仅显示，未发布）'}")
                self.last = s["state"]
            if mono() - self.t_rep >= 5:
                self.get_logger().info(stats.report())
                self.t_rep = mono()

    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="0", help="摄像头编号，或用来联调的视频文件")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=float, default=15.0, help="发给电脑的帧率上限")
    ap.add_argument("--quality", type=int, default=80, help="JPEG 质量 1~100")
    ap.add_argument("--bind", default="0.0.0.0", help="监听地址；建议改成车的 WiFi 地址 192.168.5.100")
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--stale-sec", type=float, default=0.5, help="结果距拍摄时刻超过这么久就当作 stale")
    ap.add_argument("--rate", type=float, default=20.0, help="话题发布频率")
    ap.add_argument("--enable-control", action="store_true",
                    help="接入运动控制：订阅 /traffic_light_req，请求期间发布 /traffic_light（默认关，只发验证话题）")
    ap.add_argument("--no-ros", action="store_true", help="不发布 ROS 话题，只在终端打印（电脑上联调用）")
    ap.add_argument("--selftest", action="store_true", help="只测采集和编码负载")
    ap.add_argument("--selftest-seconds", type=float, default=10.0)
    a = ap.parse_args()

    latest, res, stats, stop = Latest(), ResultState(a.stale_sec), Stats(), threading.Event()
    cur = {"ep": 0, "conn": None}
    threading.Thread(target=capture_loop, args=(a, latest, stats, stop), daemon=True).start()
    if a.selftest:
        selftest(a, latest, stats)
        stop.set()
        return
    threading.Thread(target=server_loop, args=(a, latest, res, stats, stop, cur), daemon=True).start()
    if a.no_ros:
        console_loop(a, res, stats, stop)
    else:
        ros_loop(a, res, stats, stop)


if __name__ == "__main__":
    main()
