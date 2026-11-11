#!/usr/bin/env python3
"""手机遥控红绿灯：小车开个网页，手机点按钮代替视觉识别。

⚠️ 仅用于练习/调试。正式比赛请用视觉识别，人工点按钮通常算违规。

它做什么：
    手机浏览器 ──HTTP──► 小车上的本服务 ──ROS──► /traffic_light = 1
                                                      ↓
                                              pure_track_test 收到就走

为什么不用改任何现有代码：
    pure_track_test 本来就在订阅 /traffic_light（视觉那边认出绿灯也发这个话题），
    所以这里只要往同一个话题发一条消息就行。

怎么用（小车上）：
    source /opt/ros/humble/setup.bash
    /usr/bin/python3 light_web_btn.py
    # 手机浏览器打开 http://<小车IP>:8000

    # 换端口
    /usr/bin/python3 light_web_btn.py --port 8080

    # 看小车 IP
    hostname -I

两个前提（少一个就点不动）：
    1. 车必须停在红绿灯点，pure_track_test 处于"等待中"（is_waiting_traffic_light=1）。
       没停住的时候点按钮不会有任何反应 —— 这是 pure_track_test 那边设计的。
    2. 手机和小车在同一局域网。
"""
import argparse
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PAGE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title></title>
<style>
  * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
  html, body {
    margin: 0; padding: 0; height: 100%;
    background: #ffffff;
    font-family: -apple-system, "PingFang SC", "Microsoft YaHei", sans-serif;
    user-select: none; overflow: hidden;
  }
  /* 整块白色区域：看着像白屏，但可以点（发"绿"） */
  #go {
    display: flex; width: 100%; height: calc(100% - 64px);
    align-items: center; justify-content: center;
    border: 0; background: #ffffff; color: #b4b4b4;
    font-size: clamp(15px, 4.2vw, 19px); font-weight: 400;
    letter-spacing: 1px; cursor: pointer;
    outline: none;
  }
  #go:active { background: #fbfbfb; }
  /* 底部灰色条（原红色按钮） */
  #red {
    display: block; width: 100%; height: 64px;
    border: 0; border-top: 1px solid #f0f0f0;
    background: #f7f7f7; color: transparent;
    font-size: 0; cursor: pointer; outline: none;
  }
  #red:active { background: #efefef; }
</style>
</head>
<body>
  <button id="go">网络异常无法访问</button>
  <button id="red"></button>

<script>
const go = document.getElementById('go');
const red = document.getElementById('red');

async function send(value) {
  try {
    await fetch('/send?value=' + value + '&t=' + Date.now(), { cache: 'no-store' });
  } catch (e) {
    /* 静默：页面上不显示任何提示 */
  }
}

go.addEventListener('click', () => send(1));
red.addEventListener('click', () => send(0));
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    """只处理两个请求：GET / 返回页面，GET /send?value=1 发话题。"""

    server_version = "LightWebBtn/1.0"

    def log_message(self, fmt, *args):        # 手机每点一次都刷日志太吵，只记发送
        pass

    def _reply(self, code, body, ctype="application/json; charset=utf-8"):
        data = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = self.path.split("?")[0]

        if path in ("/", "/index.html"):
            self._reply(200, PAGE, "text/html; charset=utf-8")
            return

        if path == "/send":
            # 从查询串里取 value
            value = None
            if "?" in self.path:
                for kv in self.path.split("?", 1)[1].split("&"):
                    if kv.startswith("value="):
                        try:
                            value = int(kv.split("=", 1)[1])
                        except ValueError:
                            value = None
            if value not in (0, 1):
                self._reply(400, json.dumps({"ok": False, "error": "value 必须是 0 或 1"}))
                return
            ok, err = self.server.publish_value(value)
            self.server.log_line(f"{self.client_address[0]} 点了 {'绿灯' if value else '红灯'}")
            self._reply(200 if ok else 500,
                        json.dumps({"ok": ok, "value": value, "error": err}))
            return

        if path == "/ping":
            self._reply(200, json.dumps({"ok": True}))
            return

        self._reply(404, json.dumps({"ok": False, "error": "not found"}))

    def do_POST(self):                        # 不支持，手机端只用 GET
        self._reply(405, json.dumps({"ok": False, "error": "use GET"}))


class BtnServer(ThreadingHTTPServer):
    daemon_threads = True                     # 手机断开别卡住主线程
    allow_reuse_address = True

    def __init__(self, addr, node):
        super().__init__(addr, Handler)
        self.node = node

    def publish_value(self, value):
        return self.node.publish(value)

    def log_line(self, msg):
        self.node.get_logger().info(msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--host", default="0.0.0.0", help="监听地址，0.0.0.0 = 允许手机访问")
    ap.add_argument("--topic", default="traffic_light", help="要发布的话题名")
    args, _ = ap.parse_known_args()

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Int32

    class BtnNode(Node):
        def __init__(self):
            super().__init__("light_web_btn")
            self.pub = self.create_publisher(Int32, args.topic, 10)
            self.lock = threading.Lock()
            # 顺便订阅一下，页面上能看到有没有人在发
            self.create_subscription(Int32, args.topic, self._on_echo, 10)

        def _on_echo(self, msg):
            pass                              # 目前不用，留着扩展

        def publish(self, value):
            """从 HTTP 线程调用，加锁保证线程安全。"""
            try:
                with self.lock:
                    msg = Int32()
                    msg.data = int(value)
                    self.pub.publish(msg)
                return True, None
            except Exception as e:            # 不能因为发消息失败把服务搞挂
                return False, f"{type(e).__name__}: {e}"

    # 静默模式：ROS 只报错误，正常启动不刷日志
    rclpy.init(args=["--ros-args", "--log-level", "error"])
    node = BtnNode()

    try:
        srv = BtnServer((args.host, args.port), node)
    except OSError as e:
        print(f"启动失败：端口 {args.port} 起不来（{e}）", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    print("OK", flush=True)          # 成功只有这一行

    # ROS 在后台线程转，HTTP 在主线程服务
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
