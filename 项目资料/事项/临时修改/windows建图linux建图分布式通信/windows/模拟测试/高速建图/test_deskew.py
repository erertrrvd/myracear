"""假数据验证 scan_tf_bridge 去畸变：10x8 m 方形场地，车 v=3 m/s、w=0.8 rad/s，一圈 0.1 s，
时间戳在一圈结束。对比：理想扫描（全部点都在结束时刻测）vs 去畸变后 / 不去畸变。"""
import math, sys, types, importlib.util, importlib.machinery
from types import SimpleNamespace

path = r'C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\scripts\scan_tf_bridge'
loader = importlib.machinery.SourceFileLoader('bridge', path)
spec = importlib.util.spec_from_loader('bridge', loader)
m = importlib.util.module_from_spec(spec); loader.exec_module(m)

W, H = 10.0, 8.0
def ray(px, py, a):
    """从 (px,py) 沿角 a 打到矩形 [0,W]x[0,H] 墙的距离。"""
    c, s = math.cos(a), math.sin(a); best = 1e9
    for t in ([(0 - px) / c, (W - px) / c] if abs(c) > 1e-9 else []) + ([(0 - py) / s, (H - py) / s] if abs(s) > 1e-9 else []):
        if t > 1e-6:
            x, y = px + t * c, py + t * s
            if -1e-6 <= x <= W + 1e-6 and -1e-6 <= y <= H + 1e-6: best = min(best, t)
    return best

v, w, T, N = 3.0, 0.8, 0.1, 980
x1, y1, th1 = 5.0, 4.0, 0.3            # 一圈结束（时间戳）时刻的位姿
def pose(dt):                           # dt<=0：结束前 dt 时刻的位姿（恒速圆弧近似）
    th = th1 + w * dt
    return x1 + v * dt * math.cos(th1 + w * dt / 2), y1 + v * dt * math.sin(th1 + w * dt / 2), th

def scan(ideal):
    rs = []
    for i in range(N):
        a = i * 2 * math.pi / (N - 1)
        dt = 0.0 if ideal else (i / N - 1.0) * T
        px, py, th = pose(dt)
        rs.append(ray(px, py, th + a))
    return SimpleNamespace(ranges=rs, angle_min=0.0, angle_max=2 * math.pi, range_min=0.0, range_max=200.0,
                           scan_time=T, header=SimpleNamespace(stamp=None))

class Fake: pass
def run(deskew, end):
    b = Fake(); b.beams = 720; b.deskew = deskew; b.stamp_at_end = end; b.scan_period = T; b.laser_frame = 'l'
    m.LaserScan = lambda: SimpleNamespace(header=SimpleNamespace())
    return m.ScanTfBridge.resample(b, scan(False), v, w).ranges

ideal = run(False, True) if False else None
b = Fake(); b.beams = 720; b.deskew = False; b.stamp_at_end = True; b.scan_period = T; b.laser_frame = 'l'
m.LaserScan = lambda: SimpleNamespace(header=SimpleNamespace())
ref = m.ScanTfBridge.resample(b, scan(True), 0, 0).ranges
for name, d, e in (('不去畸变', False, True), ('去畸变 stamp_at_end=true', True, True), ('去畸变 设反(false)', True, False)):
    out = run(d, e)
    err = sorted(abs(a - r) for a, r in zip(out, ref) if math.isfinite(a) and math.isfinite(r))
    print('%-26s 中位误差 %.3f m  95%% %.3f m  最大 %.3f m' % (name, err[len(err) // 2], err[int(.95 * len(err))], err[-1]))
