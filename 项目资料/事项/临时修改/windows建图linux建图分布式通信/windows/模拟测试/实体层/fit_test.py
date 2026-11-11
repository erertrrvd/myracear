import ast, math, sys
import numpy as np
src = open(sys.argv[1], encoding='utf-8').read()
fn = next(n for n in ast.parse(src).body if isinstance(n, ast.FunctionDef) and n.name == 'fit_circle')
ns = {'np': np, 'math': math}
exec(compile(ast.Module([fn], []), 'f', 'exec'), ns)
rng = np.random.default_rng(0)
for arc in (60, 120, 180):
    rs = []
    for _ in range(200):
        a = np.radians(rng.uniform(0, 360)) + np.radians(np.linspace(-arc / 2, arc / 2, 60))
        pts = np.column_stack([0.08 * np.cos(a), 0.08 * np.sin(a)]) + rng.normal(0, 0.01, (60, 2))
        f = ns['fit_circle'](pts)
        if f:
            rs.append(f[2])
    print(f'圆弧 {arc:3d}°：拟合半径中位数 {np.median(rs):.4f}（真值 0.08），成功 {len(rs)}/200')
