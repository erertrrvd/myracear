# AB 路径拼接：改动代码汇总

这个文件夹里是 AB 停车路径拼接所改动和新增的全部代码文件（副本）。在代码里搜索 **`【AB改动`** 就能找到每处改动，每处都标了“开始 / 结束”和实现的功能。

完整的设计说明、使用流程和代码检查记录见上一级目录的 [AB路径拼接.md](../AB路径拼接.md)。

## 实现了什么功能

比赛流程：从发车区出发跑 2 圈，第 2 圈快结束时拐进 AB 停车区，停在 A 或 B 就结束，不回发车区。停 A 还是 B 提前已知。

1. **完整路径和 AB 轨迹分开录**
   - `path.csv`：用原有的 `save_path` 从发车区出发跑两圈，录到第 2 圈到达分叉点为止。
   - `ab_a.csv` / `ab_b.csv`：用新增的 `save_ab_path` 节点，把车放在分叉点单独录。录出来直接是拼接需要的格式（以分叉点为原点的相对坐标，第一个点是 0,0,0），不用再手动转换。
2. **只录一边，另一边自动镜像**：只有 `ab_a.csv` 时，B 由 A 左右翻转得到，反过来也一样。两个都没有时，用代码里原来写死的轨迹。
3. **启动时指定停 A 还是 B**：`ros2 launch my_racecar move_test.launch.py ab:=a`（或 `ab:=b`，`none` 表示不拼）。
4. **自动拼接**：`get_local_path` 启动时把 AB 轨迹按分叉点的位置和朝向旋转、平移，接在 `path.csv` 后面。AB 段速度统一 1.5，最后几个点减速、停车，停到 A/B 即结束。
5. **稳健性**：自动去掉 `path.csv` 里的重复点；AB 轨迹第一个点和分叉点重合时跳过，避免曲率异常。
6. **可选**：运行中向 `/ab_select` 发 `"a"` / `"b"` / `"none"` 也能切换（一般用不到）。

## 文件清单

| 文件 | 状态 | 内容 |
|---|---|---|
| `src/save_ab_path.cpp` | 新增 | AB 停车轨迹录制节点 |
| `launch/save_ab_path.launch.py` | 新增 | 启动里程计 + 录制节点，`ab:=a` 或 `ab:=b` |
| `src/get_local_path.cpp` | 修改 | 7 处改动，见下表 |
| `launch/move_test.launch.py` | 修改 | 新增 `ab` 启动参数，`get_local_path` 新增 AB 相关参数 |
| `CMakeLists.txt` | 修改 | 注册 `save_ab_path`（3 行） |

### `get_local_path.cpp` 的 7 处改动

| 标记 | 改了什么 | 实现的功能 |
|---|---|---|
| 【AB改动1】 | 加 `std_msgs/msg/string.hpp` | `/ab_select` 话题要用 |
| 【AB改动2】 | `AB_stop` 新增 `mirror()`、`load()`，重写 `concat_path()` | 从 CSV 读 AB 轨迹、自动镜像、统一速度；合并 A/B 两份重复代码；跳过与分叉点重合的点 |
| 【AB改动3】 | 新增成员变量 | 保存原始路径、AB 轨迹和新参数 |
| 【AB改动4】 | 声明新参数 | `ab`、`ab_a_csv`、`ab_b_csv`、`ab_speed`、`laps`、`ab_fork_index` |
| 【AB改动5】 | 读取新参数 | `ab=a/b/none` 覆盖原来的 `ab_flag` |
| 【AB改动6】 | 构造函数 | 读 `path.csv` 后去重，加载 AB 轨迹，调用 `rebuild_global_path()`，订阅 `/ab_select` |
| 【AB改动7】 | 新增 `rebuild_global_path()`、`ab_select_callback()` | 组装完整路径（`path.csv` 到分叉点 + AB 段）、打速度标记、算曲率；运行中切换 A/B |

### 新增参数（`get_local_path`）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `ab` | `""`（launch 里默认 `a`） | `a` / `b` / `none`，停哪个停车位 |
| `ab_speed` | 1.5 | AB 段速度；设 0 则用 CSV 第 4 列 |
| `ab_fork_index` | -1 | 分叉点在 `path.csv` 的序号，负数从末尾倒数；-1 = 最后一个点 |
| `laps` | 1 | `path.csv` 按原样用；只录了 1 圈时设 2，会自动重复一圈 |
| `ab_a_csv` / `ab_b_csv` | `/home/racecar/src/my_racecar/include/ab_a.csv`、`ab_b.csv` | AB 轨迹文件 |

## 使用流程（每个代码块自带完整开头，复制即用，按顺序，可重复）

以下都在**小车**上（SSH：`ssh root@192.168.5.100`）。`save_path` 会**覆盖** `include/path.csv`，所以第 2 步前先自动备份。

### 1. 编译（拷入改动文件后做一次）
```bash
cd /home/racecar
source /opt/ros/humble/setup.bash
colcon build --packages-select my_racecar --symlink-install
source /home/racecar/install/local_setup.bash
```

### 2. 录完整路径（发车区出发跑两圈，到分叉点停）
```bash
cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
export ROS_DOMAIN_ID=42
cp /home/racecar/src/my_racecar/include/path.csv /home/racecar/src/my_racecar/include/path.csv.bak_$(date +%Y%m%d_%H%M%S) 2>/dev/null
ros2 launch my_racecar save_path.launch.py
```
到分叉点后 `Ctrl+C`。

### 3. 录 A 停车轨迹（车放到分叉点，车头沿路径方向）
```bash
cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
export ROS_DOMAIN_ID=42
ros2 launch my_racecar save_ab_path.launch.py ab:=a
```
开到 A 停车位后 `Ctrl+C`，生成 `include/ab_a.csv`。B 不用录，会自动镜像；想单独录 B 把 `ab:=a` 改成 `ab:=b`。

### 4. 比赛：停 A 或停 B（启动时指定）
```bash
cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
export ROS_DOMAIN_ID=42
ros2 launch my_racecar move_test.launch.py ab:=a
```
停 B 把 `ab:=a` 改成 `ab:=b`；不拼 AB 用 `ab:=none`。启动日志会打印 `AB 轨迹来源: CSV / CSV（另一侧镜像） / 代码内置` 和分叉点序号。

### 5. 想先在电脑上看路径是否对（可选）
见 `..\csv路径和slam融合\README.md`。

## 注意

- 这里的副本加了改动标记注释，**`get_local_path.cpp` 已转成 UTF-8**（原项目里是 GBK）。拷回项目直接替换也能编译，只是编码会变。
- AB 录制起点必须和 `path.csv` 末点（分叉点）的位置、朝向一致，否则 AB 段整体偏。重录 `path.csv` 后 AB 也要重录。
- 还没在小车上编译验证，拷上车后先 `colcon build`。
