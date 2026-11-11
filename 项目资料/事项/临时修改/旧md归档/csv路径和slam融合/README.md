# CSV 路径 与 SLAM 地图 融合显示（rviz2，离线查看，不做拟合）

## 实现了什么
把 **已保存的 SLAM 地图**（`map.yaml` + `map.pgm`）和 **CSV 路径**（`path.csv`、A/B 停车轨迹、任意调试 CSV）放进同一个 rviz2 里看。
- 不需要小车、不需要实时 SLAM；没有地图时也能只看 CSV 路径。
- 路径按 CSV 里的坐标原样放进 `map` 坐标系，不自动拟合；对不上时可手动给 `dx / dy / dyaw`（米、弧度）。
- 调试 CSV（`extra_csv`）：多个文件、每个一种颜色，起点绿球、终点红球，标文件名和点数，**文件改动后 1 秒内自动刷新**。
- 没有 SLAM 发 TF 时，节点自己发一个静态 `odom -> map` 单位变换，否则 rviz2 找不到 `map` 坐标系会什么都不画（参数 `publish_tf`，真 SLAM 在跑时设 `false`）。

## 文件清单
| 备份位置 | 对应的实际位置 | 说明 |
|---|---|---|
| `mapping/scripts/view_path_on_map` | 项目 `mapping/scripts/view_path_on_map` | 新增。Python 节点：读 pgm/yaml 发地图，读 CSV 发路径/调试段/分叉点/静态 TF |
| `mapping/launch/view_path_on_map.launch.py` | 项目 `mapping/launch/` | 新增。Ubuntu/小车上用的 launch（节点 + rviz2） |
| `mapping/rviz/path_on_map.rviz` | 项目 `mapping/rviz/` | 新增。rviz2 显示配置（地图、完整路径、A、B、分叉点、调试段） |
| `CMakeLists.diff.txt` | 项目 `CMakeLists.txt` | 修改：`install(PROGRAMS ...)` 里加一行 `mapping/scripts/view_path_on_map` |
| `windows_D_ubuntu_slam/view_csv.ps1`、`view_csv.bat` | `D:\ubuntu\slam\` | 新增。Windows 一键启动（节点 + rviz2），**项目路径写死在 ps1 开头的 `$proj`**，挪了项目要改 |

话题（均为 transient_local，rviz2 后开也能看到）：`/view/map`、`/view/path`、`/view/ab_a`、`/view/ab_b`、`/view/fork`、`/view/extra`。
显示颜色：`path.csv` 绿、A 橙、B 蓝、分叉点洋红。

## 拖拽使用（最省事）：把 CSV 拖到「拖拽CSV到这里.bat」
**固定地图**：本文件夹里的 `run_20261005_1552_filtered\run1\map.yaml`（以后都用这张）。

用法：
1. 在资源管理器里把一个或多个 `.csv` 文件**拖到 `拖拽CSV到这里.bat` 上**松手。
2. 自动弹出 rviz2，地图 + 你的 CSV 融合显示；关掉 rviz2 窗口即退出。
3. 双击（不拖文件）：只显示地图 + 项目里的 `path.csv`。

显示规则：
- 每个 CSV 一种颜色，起点绿球、终点红球，旁边标文件名和点数；**改文件会自动刷新**。
- `get_local_path` 的跟踪日志（`path_tracking_*.csv`，表头 `px,py,...,cx,cy,...`）自动拆成 **“目标”“实际”两条线**，可以直接对比跟踪效果。
- 其他 CSV 按列名 `x`、`y`（没有表头就取前两列）画一条线，空行和 `#` 注释行跳过。
- 只画拖进来的 CSV，不画 `path.csv`。障碍格加粗 1 格（只影响显示，地图文件不变）。

命令行等价写法（需要时）：
```powershell
cd "D:\VSCode\ros2技术集\智能车总文件\my_racecar（复件）\事项\临时修改\csv路径和slam融合"
.\drop_view.ps1 "D:\a.csv" "D:\b.csv"
```

**注意**
- CSV 坐标必须和这张地图在同一个坐标系（以建图起点为原点）。起点相对坐标的文件（如 `ab_a.csv`，从 0,0 开始）会画在原点附近，不会自动摆到分叉点。
- 地图文件夹要和 `drop_view.ps1` 放在同一目录；查看工具用的是 `D:\ubuntu\slam\view_csv.ps1`（那里有 rviz2 的环境）。
- 想换地图：改 `drop_view.ps1` 开头的 `$mapDirName`。
- 文件：`拖拽CSV到这里.bat`（入口）、`drop_view.ps1`（实际逻辑）。

## 一、Windows 电脑（推荐，已实测可用）
环境用 `D:\ubuntu\slam`（pixi 工程，自带 rviz2、slam_toolbox）。项目里的 `mapping\windows` 那套环境不完整（缺 rviz2、numpy 的 MKL 库缺失），不要用。**不需要连小车。**

**每个代码块自带 `cd`，复制即用，可重复；关掉 rviz2 窗口后台节点自动结束。** 不同场景选一块用：

### 1. 只看 path.csv（先用这个确认能显示）
```powershell
cd D:\ubuntu\slam
.\view_csv.bat
```

### 2. 叠加调试 CSV（可多个；改文件 1 秒内自动刷新）
```powershell
cd D:\ubuntu\slam
.\view_csv.bat -Extra "D:\debug\a.csv","D:\debug\b.csv"
```

### 3. 加最新保存的地图（自动找 maps 下最新的 map.yaml）
```powershell
cd D:\ubuntu\slam
$m = (Get-ChildItem D:\ubuntu\slam\maps -Recurse -Filter map.yaml | Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName
Write-Host "地图: $m"
.\view_csv.bat -Map $m
```
障碍小、锥桶看不清：加 `-Inflate 1`（只影响显示，障碍格加粗 1 格）。叠调试 CSV：最后一行改成 `.\view_csv.bat -Map $m -Extra "D:\debug\a.csv"`。

### 4. 路径和地图对不上：手动平移/旋转（米、弧度）
```powershell
cd D:\ubuntu\slam
$m = (Get-ChildItem D:\ubuntu\slam\maps -Recurse -Filter map.yaml | Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName
.\view_csv.bat -Map $m -Dx 0.2 -Dy -0.1 -Dyaw 0.05
```

### 5. 显示 A/B 停车轨迹（只给 A，B 自动镜像）
```powershell
cd D:\ubuntu\slam
.\view_csv.bat -AbA "D:\path\ab_a.csv" -ForkIndex -1
```
`-ForkIndex` 是分叉点在 path.csv 的序号，负数从末尾倒数。换路径文件加 `-PathCsv "D:\other\path.csv"`。

节点日志：`%TEMP%\view_csv_node.err`。参数：`-Extra`、`-Map`、`-PathCsv`、`-AbA`、`-AbB`、`-ForkIndex`（默认 -1）、`-Dx -Dy -Dyaw`。

## 二、Linux 电脑 / 小车（ROS 2 Humble，需编译本包）
### 1. 编译（代码更新后做一次）
Linux 电脑：
```bash
cd "/data/code/智能车/智能车(1)/my_racecar（复件）(1)"
source /opt/ros/humble/setup.bash
colcon build --packages-select my_racecar
```
小车：
```bash
cd /home/racecar
source /opt/ros/humble/setup.bash
colcon build --packages-select my_racecar --symlink-install
```

### 2. 显示（Linux 电脑；地图自动取最新一次，没有地图就不会传 map_yaml）
```bash
cd "/data/code/智能车/智能车(1)/my_racecar（复件）(1)"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
MAP=$(ls -dt ~/maps/distributed_*/run1/map.yaml 2>/dev/null | head -n 1)
echo "地图: ${MAP:-无}"
ros2 launch my_racecar view_path_on_map.launch.py \
  ${MAP:+map_yaml:=$MAP} \
  extra_csv:=/路径/debug_a.csv,/路径/debug_b.csv
```
其他参数：`path_csv`、`ab_a_csv`、`ab_b_csv`、`ab_fork_index`、`concat_dy`、`dx dy dyaw`、`rviz:=false`（只起节点）、`rviz_config:=...`。
没有地图时不给 `map_yaml` 即可（日志会报一条 ERROR，路径照常显示）。

## 三、CSV 格式要求
- 有表头：按列名 `x`、`y` 取（顺序随意，有 `yaw` 也读），其余列忽略；无表头：取前两列。
- 空行和 `#` 开头的注释行自动跳过。
- 坐标必须与 `path.csv` 在同一坐标系（地图的 `map` 系）；起点相对坐标的文件（如 `ab_a.csv`）通过 `-AbA` / `ab_a_csv` 传，会按分叉点自动摆放。

## 四、地图怎么来
`mapping/scripts/save_mapping` 保存的 `run{N}/map.yaml` + `map.pgm`。本节点自己解析 pgm/yaml（不依赖 nav2_map_server），像素 205 视为未知，亮度 > 阈值为空闲、< 阈值为障碍。
能不能叠准，取决于录 `path.csv` 与建图时车是否从同一位置、同一朝向出发；否则整体偏移，用 `dx/dy/dyaw` 手动调。

## 五、调试过程中修过的坑（记录）
1. `rviz` 配置里视角类名应为 `rviz_default_plugins/TopDownOrtho`（写成 `TopDownOrthoView` 会加载失败）。
2. `-p dx:=0` 会被 ROS 当成整数，而参数声明为小数，节点启动即退出 → 脚本里统一格式化为 `0.0`。
3. 离线时没有 TF，rviz2 报 `Frame [map] does not exist`，所有消息被丢 → 节点发静态 `odom->map`。
4. Windows 这套 ROS 对空参数 `-p name:=` 解析失败 → 脚本只在有值时才传该参数。
5. `.ps1` 含中文路径，需保存为 UTF-8 **带 BOM**，否则 Windows PowerShell 5.1 读成乱码。

## 六、验证情况
- 已验证：节点在 `D:\ubuntu\slam` 的 Windows 环境能运行；发布 `path.csv` 673 个点、调试段标记；`map` 坐标系存在；rviz2 订阅了各话题（命令行检查）。
- 未验证：rviz2 画面里线条的实际显示效果；带真实地图的叠加；Ubuntu 上 `ros2 launch my_racecar ...` 路径（需先 colcon build）。

## 七、相关但不在本目录的内容
- VS Code 里看 CSV 的绘图插件（不需要 ROS）：`D:\VSCode\vscode插件脚本\CSV图像生成\`（双击 `install.bat` 安装）。

---

# 附录：路径与完整代码

## 附录 A：所有路径

| 内容 | 项目/运行位置（绝对路径） | 本备份中的位置 |
|---|---|---|
| 节点脚本 | `C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\scripts\view_path_on_map` | `mapping\scripts\view_path_on_map` |
| launch | `C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\launch\view_path_on_map.launch.py` | `mapping\launch\view_path_on_map.launch.py` |
| rviz 配置 | `C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\rviz\path_on_map.rviz` | `mapping\rviz\path_on_map.rviz` |
| CMake 改动 | `C:\Users\lzc2\Desktop\my_racecar（复件）\CMakeLists.txt`（`install(PROGRAMS` 里加一行） | `CMakeLists.diff.txt` |
| Windows 启动脚本 | `D:\ubuntu\slam\view_csv.ps1`、`D:\ubuntu\slam\view_csv.bat` | `windows_D_ubuntu_slam\` |
| Windows ROS 环境（pixi 工程） | `D:\ubuntu\slam`（`pixi.toml`，含 rviz2、slam_toolbox） | — |
| 路径文件 | `C:\Users\lzc2\Desktop\my_racecar（复件）\include\path.csv` | — |
| A/B 轨迹（录好后） | `C:\Users\lzc2\Desktop\my_racecar（复件）\include\ab_a.csv`、`ab_b.csv` | — |
| 保存的地图 | `D:\ubuntu\slam\maps\run{N}\map.yaml`（车上：`/home/racecar/maps/mapping_only/run{N}/map.yaml`） | — |
| 小车上路径文件 | `/home/racecar/src/my_racecar/include/path.csv` | — |
| 本备份目录 | `D:\VSCode\ros2技术集\智能车总文件\my_racecar（复件）\事项\临时修改\csv路径和slam融合` | — |
| CSV 绘图插件（VS Code） | `D:\VSCode\vscode插件脚本\CSV图像生成\` | — |

## 附录 B：CMakeLists.txt 改动

```diff
diff --git a/CMakeLists.txt b/CMakeLists.txt
index 3c5ffb3..cbf0e92 100644
--- a/CMakeLists.txt
+++ b/CMakeLists.txt
@@ -161,6 +161,7 @@ install(PROGRAMS
   mapping/scripts/mapping_status_logger
   mapping/scripts/save_mapping
   mapping/scripts/scan_tf_bridge
+  mapping/scripts/view_path_on_map
   DESTINATION lib/${PROJECT_NAME}
 )
```

## 附录 C：mapping/scripts/view_path_on_map（Python 节点）

```python
#!/usr/bin/python3
"""把已保存的地图（map.yaml + map.pgm）和 CSV 路径一起发布，供 rviz2 离线查看。

不做任何拟合：路径按 CSV 里的坐标原样放进 map 坐标系，只提供 dx/dy/dyaw 三个手动偏移参数。

发布（均为 transient_local，rviz 后开也能看到）：
  /view/map        nav_msgs/OccupancyGrid  保存的地图
  /view/path       nav_msgs/Path           path.csv（完整路径）
  /view/ab_a       nav_msgs/Path           A 停车轨迹（按分叉点摆放到 map 系）
  /view/ab_b       nav_msgs/Path           B 停车轨迹（缺哪边就由另一边镜像）
  /view/fork       visualization_msgs/Marker  分叉点
  /view/extra      visualization_msgs/MarkerArray  调试用额外 CSV 段（extra_csv，逗号分隔多个，
                   每个文件一种颜色，带文件名、起点绿球、终点红球；文件改动后 1 秒内自动刷新）
"""
import math
import re
import sys
import os
from pathlib import Path as FsPath

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def read_csv(path):
    """读 x,y,yaw[,v]，跳过空行、# 注释和表头。"""
    pts = []
    if not path:
        return pts
    try:
        text = FsPath(path).read_text(encoding='utf-8', errors='ignore')
    except OSError:
        return pts
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        try:
            v = [float(t) for t in line.split(',')]
        except ValueError:
            continue  # 表头
        if len(v) >= 3:
            pts.append((v[0], v[1], v[2]))
    return pts


def read_xy(path):
    """调试用：有表头就按列名 x、y 取（顺序随意，可含 yaw）；无表头取前两列。"""
    pts = []
    try:
        text = FsPath(path).read_text(encoding='utf-8', errors='ignore')
    except OSError:
        return pts
    ix, iy, iyaw = 0, 1, 2
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        try:
            v = [float(t) for t in line.split(',')]
        except ValueError:
            names = [t.strip().lower() for t in line.split(',')]
            if 'x' in names and 'y' in names:
                ix, iy = names.index('x'), names.index('y')
                iyaw = names.index('yaw') if 'yaw' in names else -1
            continue
        if len(v) > max(ix, iy):
            pts.append((v[ix], v[iy], v[iyaw] if 0 <= iyaw < len(v) else 0.0))
    return pts


def read_series(path):
    """返回 [(后缀, 点列表)]。

    get_local_path 的跟踪日志（表头 px,py,pyaw,t,index,cx,cy,cyaw）= 前面是目标轨迹 (px,py)，
    后面是实际行驶轨迹 (cx,cy)，拆成“目标”“实际”两条；其他 CSV 按 x,y（或前两列）画一条。"""
    try:
        text = FsPath(path).read_text(encoding='utf-8', errors='ignore')
    except OSError:
        return []
    names = None
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        first = [t.strip().lower() for t in line.split(',')]
        try:
            [float(t) for t in first]
        except ValueError:
            names = first
        break
    if names and {'px', 'py', 'cx', 'cy'} <= set(names):
        idx = {n: names.index(n) for n in ('px', 'py', 'pyaw', 'cx', 'cy', 'cyaw') if n in names}
        tgt, act = [], []

        def num(parts, k):
            try:
                return float(parts[idx[k]]) if k in idx and parts[idx[k]].strip() != '' else None
            except (ValueError, IndexError):
                return None

        for line in text.splitlines()[1:]:
            parts = line.split(',')
            x, y = num(parts, 'px'), num(parts, 'py')
            if x is not None and y is not None:
                tgt.append((x, y, num(parts, 'pyaw') or 0.0))
            x, y = num(parts, 'cx'), num(parts, 'cy')
            if x is not None and y is not None:
                act.append((x, y, num(parts, 'cyaw') or 0.0))
        return [(' 目标', tgt), (' 实际', act)]
    return [('', read_xy(path))]


def read_pgm(path):
    data = FsPath(path).read_bytes()
    m = re.match(rb'P5\s+(?:#[^\n]*\n\s*)*(\d+)\s+(\d+)\s+(\d+)\s', data)
    if not m:
        raise RuntimeError(f'不是 P5 格式的 pgm：{path}')
    w, h = int(m.group(1)), int(m.group(2))
    pix = data[m.end():m.end() + w * h]
    if len(pix) != w * h:
        raise RuntimeError('pgm 数据长度与尺寸不符')
    return w, h, pix


def read_map_yaml(path):
    text = FsPath(path).read_text(encoding='utf-8')
    img = re.search(r'^image:\s*(\S+)', text, re.M).group(1)
    res = float(re.search(r'^resolution:\s*(\S+)', text, re.M).group(1))
    org = [float(t) for t in re.search(r'^origin:\s*\[([^\]]+)\]', text, re.M).group(1).split(',')]
    neg = re.search(r'^negate:\s*(\d)', text, re.M)
    occ = re.search(r'^occupied_thresh:\s*(\S+)', text, re.M)
    free = re.search(r'^free_thresh:\s*(\S+)', text, re.M)
    return dict(image=img, res=res, origin=org,
                negate=int(neg.group(1)) if neg else 0,
                occ=float(occ.group(1)) if occ else 0.65,
                free=float(free.group(1)) if free else 0.196)


def mirror(pts):
    return [(x, -y, -yaw) for x, y, yaw in pts]


def place(rel, fx, fy, fyaw, dy=0.0):
    """起点相对坐标 -> 以分叉点位姿为原点放进全局系（与 get_local_path 的拼接一致）。"""
    c, s = math.cos(fyaw), math.sin(fyaw)
    out = []
    for x, y, yaw in rel:
        a = yaw + fyaw
        a = math.atan2(math.sin(a), math.cos(a))
        out.append((x * c - y * s + fx, x * s + y * c + fy + dy, a))
    return out


class Viewer(Node):
    def __init__(self):
        super().__init__('view_path_on_map')
        inc = '/home/racecar/src/my_racecar/include/'
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('path_csv', inc + 'path.csv')
        self.declare_parameter('ab_a_csv', inc + 'ab_a.csv')
        self.declare_parameter('ab_b_csv', inc + 'ab_b.csv')
        self.declare_parameter('ab_fork_index', -1)
        self.declare_parameter('concat_dy', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('extra_csv', '')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('show_path', True)    # False = 不画 path.csv / AB / 分叉点，只画 extra_csv
        self.declare_parameter('inflate_cells', 0)   # 仅显示用：把障碍格向外加粗几格（0=不加粗）
        self.declare_parameter('dx', 0.0)
        self.declare_parameter('dy', 0.0)
        self.declare_parameter('dyaw', 0.0)
        g = lambda n: self.get_parameter(n).value
        self.frame = g('frame_id')

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        if g('publish_tf'):
            # 离线查看时没有 SLAM 在发 TF，rviz2 找不到 map 坐标系就什么都不画；
            # 这里发一个静态 odom->map 单位变换让 map 存在。接着真 SLAM 用时设 publish_tf:=false
            from tf2_ros import StaticTransformBroadcaster
            self.tf_static = StaticTransformBroadcaster(self)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = self.frame
            t.transform.rotation.w = 1.0
            self.tf_static.sendTransform(t)
        self.pub_map = self.create_publisher(OccupancyGrid, '/view/map', qos)
        self.pub_path = self.create_publisher(Path, '/view/path', qos)
        self.pub_a = self.create_publisher(Path, '/view/ab_a', qos)
        self.pub_b = self.create_publisher(Path, '/view/ab_b', qos)
        self.pub_fork = self.create_publisher(Marker, '/view/fork', qos)
        self.pub_extra = self.create_publisher(MarkerArray, '/view/extra', qos)

        self.inflate = int(g('inflate_cells'))
        self.publish_map(g('map_yaml'))

        base = read_csv(g('path_csv')) if g('show_path') else []
        if not base and g('show_path'):
            self.get_logger().error(f'读不到路径：{g("path_csv")}')
        a = read_csv(g('ab_a_csv'))
        b = read_csv(g('ab_b_csv'))
        a_ok, b_ok = len(a) >= 2, len(b) >= 2
        if a_ok and not b_ok:
            b = mirror(a)
        elif b_ok and not a_ok:
            a = mirror(b)
        elif not a_ok and not b_ok:
            self.get_logger().warn('没有 ab_a.csv / ab_b.csv，只显示完整路径')
            a = b = []

        # 手动偏移（默认 0，即原样显示）
        off = (g('dx'), g('dy'), g('dyaw'))
        co, so = math.cos(off[2]), math.sin(off[2])
        shift = lambda pts: [(x * co - y * so + off[0], x * so + y * co + off[1],
                              math.atan2(math.sin(yaw + off[2]), math.cos(yaw + off[2])))
                             for x, y, yaw in pts]

        self.publish_path(self.pub_path, shift(base))
        if base and a:
            n = len(base)
            idx = g('ab_fork_index')
            idx = n + idx if idx < 0 else idx
            idx = min(max(idx, 0), n - 1)
            fx, fy, fyaw = base[idx]
            self.get_logger().info(f'分叉点 index={idx} ({fx:.2f}, {fy:.2f}, yaw={fyaw:.3f})')
            self.publish_path(self.pub_a, shift(place(a, fx, fy, fyaw, g('concat_dy'))))
            self.publish_path(self.pub_b, shift(place(b, fx, fy, fyaw, g('concat_dy'))))
            self.publish_fork(shift([(fx, fy, fyaw)])[0])
        self.shift = shift
        self.extra = [t.strip() for t in str(g('extra_csv')).split(',') if t.strip()]
        self.extra_mtime = {}
        if self.extra:
            self.publish_extra()
            self.create_timer(1.0, self.check_extra)
        self.get_logger().info('已发布，rviz2 里订阅 /view/map /view/path /view/ab_a /view/ab_b')

    def check_extra(self):
        for f in self.extra:
            try:
                if os.stat(f).st_mtime != self.extra_mtime.get(f):
                    self.publish_extra()
                    return
            except OSError:
                pass

    def publish_extra(self):
        palette = [(1.0, 0.1, 0.1), (1.0, 0.9, 0.0), (0.0, 1.0, 1.0), (1.0, 0.4, 1.0), (1.0, 0.6, 0.0)]
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        mid = 1
        k = 0                                   # 线的序号，决定颜色
        from geometry_msgs.msg import Point
        for f in self.extra:
            try:
                self.extra_mtime[f] = os.stat(f).st_mtime
            except OSError:
                self.get_logger().warn(f'找不到 extra_csv：{f}')
                continue
            series = [(suffix, self.shift(pts)) for suffix, pts in read_series(f)]
            if not any(pts for _, pts in series):
                self.get_logger().warn(f'extra_csv 没有可用的 x,y 数据：{f}')
                continue
            for suffix, pts in series:
                if not pts:
                    continue
                r, g_, b = palette[k % len(palette)]

                def mk(kind, scale, color, ident):
                    m = Marker()
                    m.header.frame_id = self.frame
                    m.ns, m.id, m.type, m.action = f'extra{k}', ident, kind, Marker.ADD
                    m.pose.orientation.w = 1.0
                    m.scale.x, m.scale.y, m.scale.z = scale
                    m.color.r, m.color.g, m.color.b, m.color.a = color
                    return m

                line = mk(Marker.LINE_STRIP, (0.06, 0.0, 0.0), (r, g_, b, 1.0), mid)
                dots = mk(Marker.SPHERE_LIST, (0.08, 0.08, 0.08), (r, g_, b, 1.0), mid + 1)
                for x, y, _ in pts:
                    pt = Point()
                    pt.x, pt.y = x, y
                    line.points.append(pt)
                    dots.points.append(pt)
                start = mk(Marker.SPHERE, (0.3, 0.3, 0.3), (0.0, 0.8, 0.0, 1.0), mid + 2)
                start.pose.position.x, start.pose.position.y = pts[0][0], pts[0][1]
                end = mk(Marker.SPHERE, (0.3, 0.3, 0.3), (0.9, 0.0, 0.0, 1.0), mid + 3)
                end.pose.position.x, end.pose.position.y = pts[-1][0], pts[-1][1]
                label = mk(Marker.TEXT_VIEW_FACING, (0.0, 0.0, 0.3), (1.0, 1.0, 1.0, 1.0), mid + 4)
                label.text = f'{FsPath(f).name}{suffix} ({len(pts)}点)'
                label.pose.position.x, label.pose.position.y, label.pose.position.z = pts[0][0], pts[0][1], 0.5
                arr.markers += [line, dots, start, end, label]
                mid += 5
                k += 1
                self.get_logger().info(f'调试段 {FsPath(f).name}{suffix}: {len(pts)} 点，起点 ({pts[0][0]:.2f}, {pts[0][1]:.2f})')
        self.pub_extra.publish(arr)

    def publish_map(self, yaml_path):
        if not yaml_path:
            self.get_logger().error('必须指定 map_yaml（保存的 map.yaml）')
            return
        info = read_map_yaml(yaml_path)
        img = FsPath(info['image'])
        if not img.is_absolute():
            img = FsPath(yaml_path).parent / img
        w, h, pix = read_pgm(img)
        data = []
        for gy in range(h):                      # 栅格第 0 行 = pgm 最下面一行
            row = (h - 1 - gy) * w
            for gx in range(w):
                p = pix[row + gx]
                occ = p / 255.0 if info['negate'] else (255 - p) / 255.0
                if occ > info['occ']:
                    data.append(100)
                elif occ < info['free']:
                    data.append(0)
                else:
                    data.append(-1)
        if self.inflate > 0:
            # 只为了看得清：把每个障碍格周围 inflate 圈也画成障碍（锥桶只有 1~3 格，不加粗很难看见）
            occ = [i for i, v in enumerate(data) if v == 100]
            k = self.inflate
            for i in occ:
                gy, gx = divmod(i, w)
                for dy in range(-k, k + 1):
                    for dx in range(-k, k + 1):
                        yy, xx = gy + dy, gx + dx
                        if 0 <= yy < h and 0 <= xx < w:
                            data[yy * w + xx] = 100
            self.get_logger().info(f'显示加粗 {k} 格（只影响显示，地图文件没改）')
        m = OccupancyGrid()
        m.header.frame_id = self.frame
        m.info.resolution = info['res']
        m.info.width, m.info.height = w, h
        m.info.origin.position.x, m.info.origin.position.y = info['origin'][0], info['origin'][1]
        yaw = info['origin'][2]
        m.info.origin.orientation.z = math.sin(yaw / 2)
        m.info.origin.orientation.w = math.cos(yaw / 2)
        m.data = data
        self.pub_map.publish(m)
        self.get_logger().info(f'地图 {w}x{h}，分辨率 {info["res"]} m，来自 {yaml_path}')

    def publish_path(self, pub, pts):
        msg = Path()
        msg.header.frame_id = self.frame
        for x, y, yaw in pts:
            p = PoseStamped()
            p.header.frame_id = self.frame
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.orientation.z = math.sin(yaw / 2)
            p.pose.orientation.w = math.cos(yaw / 2)
            msg.poses.append(p)
        pub.publish(msg)

    def publish_fork(self, pose):
        m = Marker()
        m.header.frame_id = self.frame
        m.ns, m.id, m.type, m.action = 'fork', 0, Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y = pose[0], pose[1]
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 1.0
        self.pub_fork.publish(m)


def main():
    rclpy.init(args=sys.argv)
    node = Viewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

## 附录 D：mapping/launch/view_path_on_map.launch.py

```python
"""离线查看：已保存的地图 + path.csv / AB 轨迹，一起显示在 rviz2（不做拟合）。

  ros2 launch my_racecar view_path_on_map.launch.py \
      map_yaml:=/home/racecar/maps/mapping_only/run1/map.yaml

调试：extra_csv:=/路径/a.csv,/路径/b.csv 把任意 CSV 段高亮叠在图上（改文件会自动刷新）。
路径位置不对时，用 dx / dy / dyaw 手动平移旋转（米、弧度），例如 dx:=0.2 dyaw:=0.05。
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('my_racecar'))
    inc = '/home/racecar/src/my_racecar/include/'
    args = {
        'map_yaml': '',
        'path_csv': inc + 'path.csv',
        'ab_a_csv': inc + 'ab_a.csv',
        'ab_b_csv': inc + 'ab_b.csv',
        'ab_fork_index': '-1',
        'concat_dy': '0.0',
        'extra_csv': '',
        'inflate_cells': '0',     # 仅显示：障碍格加粗几格
        'dx': '0.0', 'dy': '0.0', 'dyaw': '0.0',
        'rviz': 'true',
        'rviz_config': str(share / 'mapping' / 'rviz' / 'path_on_map.rviz'),
    }
    params = {k: LaunchConfiguration(k) for k in
              ('map_yaml', 'path_csv', 'ab_a_csv', 'ab_b_csv', 'concat_dy', 'extra_csv', 'inflate_cells', 'dx', 'dy', 'dyaw')}
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()] + [
            Node(package='my_racecar', executable='view_path_on_map', output='screen',
                 parameters=[params, {'ab_fork_index': LaunchConfiguration('ab_fork_index')}]),
            Node(package='rviz2', executable='rviz2', output='screen',
                 arguments=['-d', LaunchConfiguration('rviz_config')],
                 condition=IfCondition(LaunchConfiguration('rviz'))),
        ])
```

## 附录 E：mapping/rviz/path_on_map.rviz

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Name: Grid
    - Alpha: 0.8
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: true
      Enabled: true
      Name: 保存的地图
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /view/map
      Use Timestamp: false
    - Alpha: 1
      Class: rviz_default_plugins/Path
      Color: 25; 255; 0
      Enabled: true
      Line Style: Lines
      Line Width: 0.04
      Name: path.csv 完整路径
      Pose Style: None
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /view/path
    - Alpha: 1
      Class: rviz_default_plugins/Path
      Color: 255; 170; 0
      Enabled: true
      Line Style: Lines
      Line Width: 0.05
      Name: A 停车轨迹
      Pose Style: None
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /view/ab_a
    - Alpha: 1
      Class: rviz_default_plugins/Path
      Color: 0; 170; 255
      Enabled: true
      Line Style: Lines
      Line Width: 0.05
      Name: B 停车轨迹
      Pose Style: None
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /view/ab_b
    - Class: rviz_default_plugins/Marker
      Enabled: true
      Name: 分叉点
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /view/fork
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: 调试 CSV 段
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /view/extra
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Name: Current View
      Scale: 40
      Target Frame: <Fixed Frame>
      X: 0
      Y: 0
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
```

## 附录 F：D:\ubuntu\slam\view_csv.ps1（Windows 一键启动）

```powershell
# 一键：在 rviz2 里显示 path.csv + 任意调试 CSV（+ 可选已保存的地图），不需要小车和 SLAM。
# 用法（PowerShell）：
#   .\view_csv.ps1                                   只看 path.csv
#   .\view_csv.ps1 -Extra "D:\a.csv","D:\b.csv"      叠加调试段（改文件自动刷新）
#   .\view_csv.ps1 -Map "D:\ubuntu\slam\maps\run1\map.yaml"   再加地图
#   .\view_csv.ps1 -Dx 0.2 -Dy -0.1 -Dyaw 0.05       手动平移/旋转路径
param(
    [string[]]$Extra = @(),
    [string]$Map = '',
    [string]$PathCsv = '',
    [string]$AbA = '',
    [string]$AbB = '',
    [int]$ForkIndex = -1,
    [int]$Inflate = 0,
    [switch]$NoPath,
    [double]$Dx = 0, [double]$Dy = 0, [double]$Dyaw = 0
)
$ErrorActionPreference = 'Stop'
$proj  = 'C:\Users\lzc2\Desktop\my_racecar（复件）'
$slam  = 'D:\ubuntu\slam'
$node  = Join-Path $proj 'mapping\scripts\view_path_on_map'
$rviz  = Join-Path $proj 'mapping\rviz\path_on_map.rviz'
if (-not $PathCsv) { $PathCsv = Join-Path $proj 'include\path.csv' }

$rosArgs = @('--ros-args', '-p', "path_csv:=$PathCsv", '-p', "ab_fork_index:=$ForkIndex",
             '-p', ('dx:=' + [string]::Format([Globalization.CultureInfo]::InvariantCulture,'{0:0.0####}',$Dx)),
             '-p', ('dy:=' + [string]::Format([Globalization.CultureInfo]::InvariantCulture,'{0:0.0####}',$Dy)),
             '-p', ('dyaw:=' + [string]::Format([Globalization.CultureInfo]::InvariantCulture,'{0:0.0####}',$Dyaw)))
if ($Extra.Count) { $rosArgs += @('-p', ('extra_csv:=' + ($Extra -join ','))) }
if ($Map)  { $rosArgs += @('-p', "map_yaml:=$Map") }
if ($NoPath) { $rosArgs += @('-p', 'show_path:=false') }   # 只显示 -Extra 的 CSV，不画 path.csv
if ($Inflate -gt 0) { $rosArgs += @('-p', "inflate_cells:=$Inflate") }   # 仅显示：障碍格加粗，锥桶更醒目
if ($AbA)  { $rosArgs += @('-p', "ab_a_csv:=$AbA") }
if ($AbB)  { $rosArgs += @('-p', "ab_b_csv:=$AbB") }

$env:ROS_LOCALHOST_ONLY = '1'        # 只在本机通信，不干扰小车网络
Set-Location $slam
$nodeProc = Start-Process pixi -ArgumentList (@('run','--frozen','python',"`"$node`"") + $rosArgs) -PassThru -WindowStyle Hidden -RedirectStandardError "$env:TEMP\view_csv_node.err" -RedirectStandardOutput "$env:TEMP\view_csv_node.out"
Start-Sleep -Seconds 3
if ($nodeProc.HasExited) { Write-Host 'Node failed to start:'; Get-Content "$env:TEMP\view_csv_node.err"; exit 1 }
try {
    pixi run --frozen rviz2 -d $rviz
} finally {
    if ($nodeProc -and -not $nodeProc.HasExited) { taskkill /PID $nodeProc.Id /T /F | Out-Null }
}
```

## 附录 G：D:\ubuntu\slam\view_csv.bat

```bat
@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0view_csv.ps1" %*
```

