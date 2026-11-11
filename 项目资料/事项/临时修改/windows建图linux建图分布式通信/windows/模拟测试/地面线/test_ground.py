"""模拟测试 GroundLineFilter：
A 操场：雷达前倾，车前 3 m 一条地面线（雷达系固定）+ 1.3 m 间距锥桶，车直行 + 转弯 → 应认出地面线、删掉线上 / 线外点、锥桶（线内）保留。
B 走廊：两侧平行墙 + 前方 6 m 一堵墙，雷达水平，车直行 4 m → 不应认出任何地面线。
C 静止：车不动，地面线一直在 → 学不出来（需要 fixed）。
"""
import importlib.machinery, importlib.util, math, sys
import numpy as np

path = r"C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\scripts\scan_tf_bridge"
loader = importlib.machinery.SourceFileLoader('stb', path)
spec = importlib.util.spec_from_loader('stb', loader)
m = importlib.util.module_from_spec(spec)
loader.exec_module(m)

N = 720
ANG = -math.pi + 2 * math.pi / N * np.arange(N)
rng = np.random.default_rng(0)


def raycast(pose, cones, walls, ground):
    """pose 世界系；cones [(x,y,r)]；walls [((x1,y1),(x2,y2))] 世界系；ground (theta, rho) 雷达系或 None"""
    x, y, yaw = pose
    r = np.full(N, np.inf)
    dirs = np.stack([np.cos(ANG + yaw), np.sin(ANG + yaw)], 1)
    for cx, cy, cr in cones:
        ox, oy = cx - x, cy - y
        b = dirs[:, 0] * ox + dirs[:, 1] * oy
        c = ox * ox + oy * oy - cr * cr
        disc = b * b - c
        t = b - np.sqrt(np.maximum(disc, 0))
        ok = (disc > 0) & (t > 0)
        r[ok] = np.minimum(r[ok], t[ok])
    for (x1, y1), (x2, y2) in walls:
        ex, ey = x2 - x1, y2 - y1
        den = dirs[:, 0] * ey - dirs[:, 1] * ex
        with np.errstate(divide='ignore', invalid='ignore'):
            t = ((x1 - x) * ey - (y1 - y) * ex) / den
            s = ((x1 - x) * dirs[:, 1] - (y1 - y) * dirs[:, 0]) / den
        ok = (np.abs(den) > 1e-9) & (t > 0) & (s >= 0) & (s <= 1)
        r[ok] = np.minimum(r[ok], t[ok])
    if ground:
        th, rho = ground
        c = np.cos(ANG - th)
        with np.errstate(divide='ignore'):
            tg = np.where(c > 1e-3, rho / c, np.inf)
        r = np.minimum(r, tg + rng.normal(0, 0.03, N))   # 地面粗糙一点
    r = r + rng.normal(0, 0.01, N)
    r[r > 12] = np.inf
    return r


def drive(segments):
    """[(v, w, 秒)] -> 10 Hz 位姿序列"""
    x = y = a = 0.0
    out = [(x, y, a)]
    for v, w, T in segments:
        for _ in range(int(T * 10)):
            a += w * 0.1
            x += v * 0.1 * math.cos(a)
            y += v * 0.1 * math.sin(a)
            out.append((x, y, a))
    return out


def run(name, poses, cones, walls, ground):
    f = m.GroundLineFilter('auto', 0.0, 0.0, 0.15, 1.0, 0.5, 1.0)
    learned_at = None
    cone_kept = cone_total = gnd_left = 0
    for k, p in enumerate(poses):
        r = raycast(p, cones, walls, ground)
        ok = np.isfinite(r)
        f.learn(r[ok] * np.cos(ANG[ok]), r[ok] * np.sin(ANG[ok]), p)
        if f.learned is not None and learned_at is None:
            learned_at = (k, f.learned)
        out = f.apply(r, ANG)
        if learned_at and ground:
            # 统计：锥桶点（不含地面）保留多少；地面点还剩多少
            rc = raycast(p, cones, walls, None)
            is_cone = np.isfinite(rc) & (np.abs(rc - r) < 0.05)
            cone_total += int(is_cone.sum())
            cone_kept += int((is_cone & np.isfinite(out)).sum())
            gnd_left += int((~is_cone & np.isfinite(out) & ok).sum())
    print(f'== {name}')
    if learned_at:
        k, c = learned_at
        print(f'  第 {k} 帧认出：theta={math.degrees(c["theta"]):.1f}° rho={c["rho"]:.2f} m along={c["along"]:.2f} rot={math.degrees(c["rot"]):.0f}°')
        print(f'  最终线：theta={math.degrees(f.line[0]):.1f}° rho={f.line[1]:.2f}')
    else:
        print('  没认出地面线')
    if learned_at and ground:
        print(f'  认出后：锥桶点保留 {cone_kept}/{cone_total}，残留的非锥桶点 {gnd_left}')
    return learned_at


# A 操场：锥桶 1.3 m 方阵，地面线法向 2°、距离 3.0 m
cones = [(1 + 1.3 * i, -3 + 1.3 * j, 0.08) for i in range(10) for j in range(5)]
A = run('A 操场（前倾，直行再转弯）', drive([(1.5, 0, 4), (1.5, 0.5, 3), (1.5, 0, 3)]), cones, [], (math.radians(2), 3.0))
# A2 慢速 0.3 m/s
A2 = run('A2 操场慢速 0.3 m/s', drive([(0.3, 0, 8)]), cones, [], (math.radians(2), 3.0))
# B 走廊：两侧墙 y=±1.2，前方 x=6 墙；雷达水平
walls = [((-5, 1.2), (20, 1.2)), ((-5, -1.2), (20, -1.2)), ((6, -1.2), (6, 1.2))]
B = run('B 走廊（雷达水平，直行 4 m 朝前墙开）', drive([(1.0, 0, 4)]), [], walls, None)
B2 = run('B2 走廊慢速 0.2 m/s 朝前墙开 3 m', drive([(0.2, 0, 15)]), [], walls, None)
# C 静止
C = run('C 静止（前倾，车不动 10 s）', drive([(0, 0, 10)]), cones, [], (math.radians(2), 3.0))
ok = bool(A) and bool(A2) and not B and not B2 and not C
print('结论：', '全部符合预期' if ok else '有不符合预期的')
sys.exit(0 if ok else 1)
