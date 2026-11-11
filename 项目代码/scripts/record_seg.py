#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
录制 AB 停车段轨迹 -> 直接写入 get_local_path.cpp 读取的 CSV（不用编译、不用转换器）

用法（就这两条）:
  python3 record_seg.py a      -> /home/racecar/src/my_racecar/include/ab_a.csv
  python3 record_seg.py b      -> /home/racecar/src/my_racecar/include/ab_b.csv

录完直接生效（重启 move_test 即可），**不需要编译，也不需要转换器**。

输出格式：x,y,yaw,v  （get_local_path.cpp 的 load_csv_file 就认这个）
  · 已自动归一化：起点平移到 (0,0)、朝向归零
    —— 因为 C++ 那边 concat_path 会把这段轨迹旋转平移到岔路口上

原始（未归一化的里程计坐标）会另存到 /home/racecar/pt/ 下面，方便回看。

不用手动 source ROS：脚本发现 import 不到 rclpy 时，会自动 source
/opt/ros/*/setup.bash 和工作空间的 setup.bash，然后用对的解释器重跑自己。

会自动把 imu_encoder_mix 和 get_distance 起起来（已经在跑就不重复起），
Ctrl+C 时一起关掉。所以一个终端就够。
"""
import math
import os
import subprocess
import sys
import time

# ================= 固定路径（想改只改这里） =================
INC = "/home/racecar/src/my_racecar/include"      # get_local_path.cpp 读的就是这里
PT = "/home/racecar/pt"                           # 原始数据（未归一化）放这
FILES = {
    "a": INC + "/ab_a.csv",
    "b": INC + "/ab_b.csv",
}
DEFAULT_KEY = "a"
DX = 0.1                     # 录制时的点距 m（和原来一样）
STEP = 0.1055                # 重采样后的点距 m —— 和 path.csv、和你原来的转换器一致
V = 2.0                      # 速度列，和原来硬编码的 2.0 一致
MIN_POINTS = 2               # 少于这么多个点就不覆盖目标文件（防止误按 Ctrl+C 清空）
# ==========================================================


def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# ---------------------------------------------------------------------------
# 下面三个函数是「AB停车轨迹转换器.html」里同名逻辑的等价实现。
# 保持和那边一致，才能保证换轨迹后结果和以前一模一样。
#   1) resample     按弧长等距重采样
#   2) yaw_from_xy  从坐标重算航向（重采样后原来的 yaw 就用不了了）
#   3) anchor       平移到起点 (0,0)、朝向归零
# ---------------------------------------------------------------------------
def arc_len(xs, ys):
    s = [0.0]
    for i in range(1, len(xs)):
        s.append(s[-1] + math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]))
    return s


def resample(xs, ys, step):
    """按弧长等距重采样。点太少就原样返回。"""
    if len(xs) < 2:
        return list(xs), list(ys)
    s = arc_len(xs, ys)
    tot = s[-1]
    if tot < step * 2:
        return list(xs), list(ys)
    m = max(1, int(round(tot / step)))
    nx, ny = [], []
    j = 0
    for k in range(m + 1):
        tgt = k * tot / m
        while j < len(s) - 2 and s[j + 1] < tgt:
            j += 1
        seg = s[j + 1] - s[j]
        u = (tgt - s[j]) / seg if seg > 1e-12 else 0.0
        nx.append(xs[j] + u * (xs[j + 1] - xs[j]))
        ny.append(ys[j] + u * (ys[j + 1] - ys[j]))
    return nx, ny


def yaw_from_xy(xs, ys):
    """从 x,y 算航向。

    内部点用中心差分；首尾点用三点圆外推（一阶前向差分会把起点的转弯量算少一半）。
    和转换器里的 yawFromXY 完全对应。
    """
    n = len(xs)
    if n == 0:
        return []
    if n == 1:
        return [0.0]
    if n == 2:
        a = math.atan2(ys[1] - ys[0], xs[1] - xs[0])
        return [a, a]

    r = [0.0] * n
    for i in range(n):
        if 0 < i < n - 1:
            r[i] = math.atan2(ys[i + 1] - ys[i - 1], xs[i + 1] - xs[i - 1])
            continue

        i0 = 0 if i == 0 else n - 3
        x1, y1 = xs[i0], ys[i0]
        x2, y2 = xs[i0 + 1], ys[i0 + 1]
        x3, y3 = xs[i0 + 2], ys[i0 + 2]

        d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
        if abs(d) < 1e-12:                      # 三点共线，退化成前后差分
            if i == 0:
                r[i] = math.atan2(y2 - y1, x2 - x1)
            else:
                r[i] = math.atan2(ys[n - 1] - ys[n - 2], xs[n - 1] - xs[n - 2])
            continue

        s1 = x1 * x1 + y1 * y1
        s2 = x2 * x2 + y2 * y2
        s3 = x3 * x3 + y3 * y3
        ux = (s1 * (y2 - y3) + s2 * (y3 - y1) + s3 * (y1 - y2)) / d
        uy = (s1 * (x3 - x2) + s2 * (x1 - x3) + s3 * (x2 - x1)) / d

        a1 = math.atan2(y1 - uy, x1 - ux)
        a3 = math.atan2(y3 - uy, x3 - ux)
        dth = wrap_pi(a3 - a1)
        tan1 = a1 + (math.pi / 2 if dth > 0 else -math.pi / 2)
        r[i] = tan1 if i == 0 else tan1 + dth
    return r


def anchor(xs, ys, yw):
    """平移到起点 (0,0)、朝向归零。

    为什么需要：录出来的是【里程计绝对坐标】，而 get_local_path.cpp 的
    concat_path 会把这段轨迹旋转+平移到岔路口上，它假设起点就是 (0,0,0)。
    """
    if not xs:
        return [], [], []
    x0, y0, a0 = xs[0], ys[0], yw[0]
    c, s = math.cos(-a0), math.sin(-a0)
    nx, ny, nw = [], [], []
    for i in range(len(xs)):
        dx, dy = xs[i] - x0, ys[i] - y0
        nx.append(c * dx - s * dy)
        ny.append(s * dx + c * dy)
        nw.append(wrap_pi(yw[i] - a0))
    return nx, ny, nw


def process_track(points, step):
    """录到的原始点 -> 可直接给 get_local_path.cpp 的轨迹。

    顺序和转换器一致：重采样 -> 从坐标重算 yaw -> 平移到原点。
    """
    if not points:
        return []
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    xs, ys = resample(xs, ys, step)
    yw = yaw_from_xy(xs, ys)
    xs, ys, yw = anchor(xs, ys, yw)
    return list(zip(xs, ys, yw))


def _build_shell(ros, rosdir, ws, cands, vers, script, args):
    """拼出要交给 bash -c 的脚本。
       必须用【换行】拼：for/then 后面不能跟 ';'，拼成一行会 syntax error。"""
    sh = []
    sh.append("source '%s'" % ros)
    if ws:
        sh.append("source '%s'" % ws)
    sh.append("export _REC_BOOTSTRAPPED=1")
    sh.append("for PY in %s; do" % " ".join("'%s'" % c for c in cands))
    sh.append('  if [ -x "$PY" ] || command -v "$PY" >/dev/null 2>&1; then')
    sh.append('    if "$PY" -c "import rclpy" >/dev/null 2>&1; then')
    sh.append('      echo "[env] 用 $PY 启动"')
    sh.append('      exec "$PY" \'%s\'%s' % (script, args))
    sh.append('    fi')
    sh.append('  fi')
    sh.append('done')
    sh.append('echo "[env] 试过的解释器都不行: %s" >&2' % " ".join(cands))
    sh.append('echo "[env] ROS 需要 python3.%s" >&2'
              % (sorted(vers, reverse=True)[0] if vers else "?"))
    sh.append('ls -d %s/lib/python3.*/site-packages 2>/dev/null >&2' % rosdir)
    sh.append('ls %s/lib/python3.*/site-packages/_rclpy_pybind11* 2>/dev/null >&2' % rosdir)
    sh.append('echo "把上面两行发我" >&2')
    sh.append('exit 1')
    return "\n".join(sh)


def _bootstrap():
    """没 rclpy 就 source 完 ROS、找到版本匹配的解释器，重跑自己。"""
    try:
        import rclpy            # noqa: F401
        return
    except ImportError:
        pass

    if os.environ.get("_REC_BOOTSTRAPPED") == "1":
        sys.stderr.write(
            "\n[错误] source 过 ROS 之后还是 import 不到 rclpy。\n"
            "       把下面这些发我：\n"
            "         ls -d /opt/ros/*/lib/python3.*/site-packages\n"
            "         ls /opt/ros/*/lib/python3.*/site-packages/_rclpy_pybind11*\n\n")
        sys.exit(1)

    ros = None
    for d in ("/opt/ros/humble", "/opt/ros/iron", "/opt/ros/jazzy", "/opt/ros/foxy"):
        if os.path.isfile(os.path.join(d, "setup.bash")):
            ros = os.path.join(d, "setup.bash")
            break
    if ros is None:
        sys.stderr.write("\n[错误] /opt/ros/ 下找不到 setup.bash —— ROS 装在哪？\n\n")
        sys.exit(1)
    rosdir = os.path.dirname(ros)

    # 从 ROS 的目录结构读出它需要哪个 python 版本（例如 python3.10）
    import glob
    import re
    vers = set()
    for pat in ("lib/python3.*/site-packages", "local/lib/python3.*/dist-packages"):
        for p in glob.glob(os.path.join(rosdir, pat)):
            m = re.search(r"python3\.(\d+)", p)
            if m:
                vers.add(m.group(1))
    cands = []
    for v in sorted(vers, reverse=True):
        cands += ["/usr/bin/python3.%s" % v, "/usr/local/bin/python3.%s" % v]
    cands += ["/usr/bin/python3", "python3"]
    seen = set()
    cands = [c for c in cands if not (c in seen or seen.add(c))]

    here = os.path.dirname(os.path.abspath(__file__))
    wss = [os.path.normpath(os.path.join(here, "..", "install", "setup.bash")),
           os.path.normpath(os.path.join(here, "..", "..", "install", "setup.bash")),
           os.path.normpath(os.path.join(here, "..", "..", "..", "install", "setup.bash")),
           "/home/racecar/install/setup.bash",
           "/home/racecar/src/my_racecar/install/setup.bash",
           os.path.expanduser("~/install/setup.bash")]
    ws = next((p for p in wss if os.path.isfile(p)), None)

    args = "".join(" '%s'" % a.replace("'", "'\\''") for a in sys.argv[1:])
    script = os.path.abspath(__file__).replace("'", "'\\''")

    shell = _build_shell(ros, rosdir, ws, cands, vers, script, args)

    print("[env] 当前 python 是 %s（%d.%d），它没有可用的 rclpy"
          % (sys.executable, sys.version_info[0], sys.version_info[1]))
    print("[env] ROS 需要 python3.%s，准备依次试: %s"
          % (sorted(vers, reverse=True)[0] if vers else "?", " ".join(cands)))
    if ws:
        print("[env] 工作空间: %s" % ws)
    sys.stdout.flush()
    os.execvp("bash", ["bash", "-c", shell])


_bootstrap()

# ---- 到这里 rclpy 一定可用 ----
import rclpy                       # noqa: E402
from rclpy.node import Node        # noqa: E402
from std_msgs.msg import Float32   # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402


def quat_to_yaw(x, y, z, w):
    # 和 save_path.cpp:157 同一个公式
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Recorder(Node):

    def __init__(self, out, raw_out, dx, kids, step=STEP):
        super().__init__("record_seg")
        self.dx = dx
        self.step = step
        self.out = out
        self.raw_out = raw_out
        self.kids = kids
        self.points = []                            # 内存里攒，结束时归一化再写

        os.makedirs(os.path.dirname(raw_out), exist_ok=True)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        self.f = open(raw_out, "w")
        self.f.write("x,y,yaw,type\n")

        self.x = self.y = self.yaw = 0.0
        self.have_odom = False
        self.last_dis = None
        self.n = 0
        self.length = 0.0
        self.px = self.py = None

        self.create_subscription(Float32, "distance", self.on_dis, 10)
        self.create_subscription(Odometry, "/encoder_imu_odom", self.on_odom, 10)
        self.get_logger().info("目标文件 : %s" % self.out)
        self.get_logger().info("原始数据 : %s" % self.raw_out)
        self.get_logger().info("点距     : %.3f m" % dx)
        self.get_logger().info("车摆在岔路口，等数据…… 开始推，Ctrl+C 结束")

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.have_odom = True

    def on_dis(self, msg):
        if not self.have_odom:
            return
        dis = msg.data
        if self.last_dis is None:
            self.last_dis = dis - self.dx          # 第一帧就记，起点不会丢
        if dis - self.last_dis < self.dx - 1e-4:
            return
        if self.px is not None:
            self.length += math.hypot(self.x - self.px, self.y - self.py)
        self.px, self.py = self.x, self.y
        self.points.append((self.x, self.y, self.yaw))
        self.f.write("%.9f,%.9f,%.9f,1\n" % (self.x, self.y, self.yaw))
        self.f.flush()                              # 边录边落盘，Ctrl+C 不丢原始数据
        self.last_dis = dis
        self.n += 1
        if self.n == 1:
            self.get_logger().info("起点已记下 (%.3f, %.3f)  朝向 %.2f°"
                                   % (self.x, self.y, math.degrees(self.yaw)))
        elif self.n % 10 == 0:
            self.get_logger().info("已录 %d 点, 长度 %.2f m" % (self.n, self.length))

    def close(self):
        try:
            self.f.close()
        except Exception:
            pass

        lg = self.get_logger()
        lg.info("=" * 52)
        lg.info("录完: %d 点, 长度 %.2f m" % (self.n, self.length))

        if len(self.points) < MIN_POINTS:
            lg.warn("点太少（< %d），不覆盖 %s" % (MIN_POINTS, self.out))
            lg.warn("原始数据还在 %s" % self.raw_out)
        else:
            # 和「AB停车轨迹转换器」完全一样的处理：
            # 重采样(0.1055) -> 从坐标重算 yaw -> 平移到原点
            pts = process_track(self.points, self.step)
            try:
                with open(self.out, "w") as f:
                    f.write("x,y,yaw,v\n")
                    for (x, y, yaw) in pts:
                        f.write("%.16f,%.16f,%.16f,%.4f\n" % (x, y, yaw, V))
                lg.info("已写入 %s" % self.out)
                lg.info("  录到 %d 点 -> 重采样成 %d 点（点距 %.4f m）"
                        % (len(self.points), len(pts), self.step))
                lg.info("  起点: (%.4f, %.4f) 朝向 %.2f°  <- 必须是 0,0,0"
                        % (pts[0][0], pts[0][1], math.degrees(pts[0][2])))
                lg.info("  终点: (%.4f, %.4f) 朝向 %.2f°"
                        % (pts[-1][0], pts[-1][1], math.degrees(pts[-1][2])))
                lg.info("  重启 move_test 就会自动读这份，不用编译")
            except OSError as e:
                lg.error("写 %s 失败: %s" % (self.out, e))

        lg.info("=" * 52)

        for p in self.kids:
            try:
                p.terminate()
                p.wait(timeout=3)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass
        if self.kids:
            lg.info("已关掉本次启动的里程计/测距节点")


def alive(pat):
    try:
        return subprocess.run(["pgrep", "-f", pat],
                              stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL).returncode == 0
    except FileNotFoundError:
        return False


def start_node(exe, pkg, kids):
    """环境已经 source 好了，直接 ros2 run"""
    if alive(exe):
        print("[node] %s 已在运行，不重复启动" % exe)
        return
    try:
        p = subprocess.Popen(["ros2", "run", pkg, exe],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        kids.append(p)
        print("[node] 已启动 %s" % exe)
    except FileNotFoundError:
        print("[node] 启动 %s 失败：找不到 ros2" % exe)


def main():
    if len(sys.argv) > 1 and sys.argv[1] in ("-h", "--help"):
        print(__doc__)
        print("可选: a | b | 任意绝对路径")
        print("可选: --step 0.1055   重采样后的点距（默认 0.1055，和 path.csv 一致）")
        return 0

    argv = [a for a in sys.argv[1:]]
    step = STEP
    if "--step" in argv:
        i = argv.index("--step")
        try:
            step = float(argv[i + 1])
            del argv[i:i + 2]
        except (IndexError, ValueError):
            sys.stderr.write("--step 后面要跟一个数字\n")
            return 1
    else:
        # 也支持 --step=0.1055 写法
        for i, a in enumerate(argv):
            if a.startswith("--step="):
                try:
                    step = float(a.split("=", 1)[1])
                    del argv[i]
                except ValueError:
                    pass
                break

    key = argv[0] if argv else DEFAULT_KEY
    out = FILES.get(key.lower(), key if os.path.isabs(key) else os.path.join(INC, key))
    stem = os.path.splitext(os.path.basename(out))[0]
    raw_out = os.path.join(PT, "%s_raw_%s.csv" % (stem, time.strftime("%m%d_%H%M%S")))

    if os.path.exists(out):
        bak = "%s.bak_%s" % (out, time.strftime("%H%M%S"))
        os.rename(out, bak)
        print("原文件已备份为 %s" % os.path.basename(bak))

    kids = []
    print("=" * 54)
    start_node("imu_encoder_mix", "encoder_imu", kids)
    start_node("get_distance", "my_racecar", kids)
    print("=" * 54)
    print("录制点距 %.3f m -> 输出重采样 %.4f m" % (DX, step))

    rclpy.init()
    node = Recorder(out, raw_out, DX, kids, step)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
