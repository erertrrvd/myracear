"""fit_track 假数据测试：每种赛道生成锥桶（噪声 0.1 m + 整体弯曲 + 15 个乱点 + 随机平移旋转 + 比例 0.97），
弯曲分两档：两头翘 0.35 m（一般漂移）和 1.2 m（大偏移）。看自动认的赛道对不对、多少锥桶被拉到线上、
校正后锥桶离真实位置多远。不用 ROS，约 1 分钟。"""
import importlib.machinery, importlib.util, json, math, tempfile
from pathlib import Path
import numpy as np

path = r'C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\scripts\fit_track'
ld = importlib.machinery.SourceFileLoader('fit_track_mod', path)
m = importlib.util.module_from_spec(importlib.util.spec_from_loader(ld.name, ld)); ld.exec_module(m)

rng = np.random.default_rng(1)
ok = total = 0
for bend in (0.35, 1.2):
    for truth in 'ABCD':
        _, tc = m.tmpl_points(m.template(truth))
        p = tc + rng.normal(0, 0.1, tc.shape)
        p[:, 1] += bend * (p[:, 0] - 8.4) ** 2 / 8.4 ** 2           # 两圈漂移：长边弯成弧，两头翘 bend m
        p = np.vstack([p, rng.uniform([-3, -3], [20, 8], (15, 2))])  # 乱点（人、树、场边）
        a = rng.uniform(-math.pi, math.pi)
        R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        p = 0.97 * p @ R.T + rng.uniform(-10, 10, 2)
        d = Path(tempfile.mkdtemp())
        (d / 'solid.json').write_text(json.dumps({'cone_centers': [[x, y, 5] for x, y in p]}))
        o = p.min(0) - 2
        W, Hh = (int(v) for v in (np.ptp(p, 0) + 4) / 0.05)
        img = np.full((Hh, W), 254, np.uint8)
        for x, y in p:
            i, j = int((x - o[0]) / 0.05), int((y - o[1]) / 0.05)
            img[Hh - 1 - j - 1:Hh - 1 - j + 2, i - 1:i + 2] = 0
        (d / 'map.pgm').write_bytes(f'P5\n{W} {Hh}\n255\n'.encode() + img.tobytes())
        (d / 'map.yaml').write_text(f'image: map.pgm\nresolution: 0.05\norigin: [{o[0]}, {o[1]}, 0]\n')
        rec = m.run(d, log=lambda s: None)
        hit = rec['track'] == truth
        ok += hit; total += 1
        print(f'弯曲 {bend} m  真值 {truth} → 认成 {rec["track"]} {"对" if hit else "错"}；分数 {rec["scores"]}；'
              f'拉到线上 {rec["matched"]}/{rec["cones"]}（真锥桶 {len(tc)} 个）；拉之前离线中位 {rec["dist_rigid_median"]} m、最远 {rec["dist_rigid_max"]} m；'
              f'图 {d / "track_fit.png"}')
print(f'结论：{ok}/{total} 认对')
