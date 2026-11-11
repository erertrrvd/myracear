# AB 路径拼接（完整路径与 AB 轨迹分开录，启动时指定停 A 还是 B）

## 比赛逻辑（规则 2.1 / 赛道 B 图）
- 从发车区出发跑 2 圈，第 2 圈快结束时拐进 AB 停车区，停在 A 或 B 就**结束**，不回发车区。
- 停 A 还是 B 提前已知，**启动时用参数 `ab` 指定**。
- 两份数据**分开录**：
  1. `path.csv`：从发车区出发完整跑两圈，**录到第 2 圈快结束、到达分叉点为止**（分叉点 = 拐进 AB 区的位置）。
  2. `ab_a.csv`（/`ab_b.csv`）：把车放到分叉点，单独录从分叉点开到 A（/B）停车位的轨迹。
- 运行时 `get_local_path` 把 AB 轨迹接在 `path.csv` 的分叉点后面。最后 `slow_point` 个点减速（v=-1）、最后 `stop_point` 个点停车（v=-2）；AB 段速度统一 `ab_speed`=1.5。

## 新增文件
| 文件 | 作用 |
|---|---|
| `src/save_ab_path.cpp` | AB 录制节点。订阅 `/encoder_imu_odom`，以第一帧位姿为原点，每 0.1m 存一点，直接输出拼接所需的**起点相对坐标** CSV（第一点 = 0,0,0），第 4 列速度 1.5 |
| `launch/save_ab_path.launch.py` | 启动里程计 + 录制节点，参数 `ab:=a` 或 `ab:=b` |
| `include/ab_a.csv` / `include/ab_b.csv` | 录制产物（**只需录一边**，另一边自动镜像） |

## 修改文件
- `CMakeLists.txt`：注册 `save_ab_path`。
- `launch/move_test.launch.py`：新增 launch 参数 `ab`（默认 `a`，可选 `a` / `b` / `none`）；`get_local_path` 新增参数 `ab_speed`、`laps`、`ab_fork_index`；launch 里不再写 `ab_flag`（节点里仍保留，`ab` 非空时覆盖它）。
- `src/get_local_path.cpp`（GBK 编码，已按 GBK 保存）：
  - `AB_stop` 新增 `load()` / `mirror()`：从 CSV 读 A、B；只有一边时另一边自动镜像（y、yaw 取反）；都没有则用代码内置轨迹；`ab_speed>0` 时 AB 段速度统一设为该值。
  - `AB_stop::concat_path` 合并 A/B 重复代码，逻辑不变（按分叉点 yaw 旋转再平移，加 `concat_dy`），加空路径保护。
  - 新增 `rebuild_global_path()`：取 `path.csv` 到 `ab_fork_index`，接上 AB 段，再打速度标记、算曲率。原构造函数的这三步改为调用它。
  - 新增订阅 `/ab_select`（`std_msgs/String`，`a`/`b`/`none`）：运行中也可改，重建全局路径（一般用不到）。
  - 新增参数：
    - `ab`：`a` / `b` / `none`，覆盖 `ab_flag`。
    - `ab_fork_index`：分叉点在 `path.csv` 的序号，默认 `-1`（最后一个点）；负数从末尾倒数。录 `path.csv` 时如果多录了一段，用它截掉多余部分，例如 `-20`。
    - `laps`：默认 1，即 `path.csv` 按原样用；若 `path.csv` 只录了 1 圈，设为 2，会自动重复整圈、最后一圈走到分叉点。
    - `ab_a_csv`、`ab_b_csv`：默认 `/home/racecar/src/my_racecar/include/ab_a.csv`、`ab_b.csv`。
    - `ab_speed`：默认 1.5。

## 使用流程
1. 编译：`colcon build --packages-select my_racecar`。
2. 录 `path.csv`：发车区出发，跑完两圈，第 2 圈到分叉点就停（原有流程 `save_path.launch.py`）。
3. 录 AB 轨迹：把车放到分叉点，车头沿路径方向：
   ```bash
   ros2 launch my_racecar save_ab_path.launch.py ab:=a
   ```
   开到 A 停车位，Ctrl+C，生成 `include/ab_a.csv`。B 不用录，自动镜像；也可 `ab:=b` 单独录。
4. 比赛，启动时指定停哪个：
   ```bash
   ros2 launch my_racecar move_test.launch.py ab:=a      # 或 ab:=b
   ```
   日志会打印 `AB 轨迹来源: CSV / CSV（另一侧镜像） / 代码内置` 和分叉点序号、坐标。

## 注意
- **AB 录制起点必须和 `path.csv` 的分叉点位置、朝向一致**（把车放到 `path.csv` 末点位置，车头方向同路径），否则 AB 段整体偏。重录了 `path.csv` 或改了 `ab_fork_index`，AB 也要重录。
- `ab_*.csv` 第 4 列速度会被 `ab_speed` 覆盖；不想覆盖就把 `ab_speed` 设为 0。
- `pure_track_test` 的 `first_stop_index` / `second_stop_index` / `traffic_light_index` 按拼接后的下标计数，当前 launch 里都是 800（不触发），要用时按实际路径长度重算。
- 当前 `path.csv`（673 点）末点是 (-5.47, -0.27)，且最后两行重复，按上面流程重录后会被替换。
- 未在 Windows 上编译验证（没有 ROS 环境），上车后先 `colcon build`。

## 代码检查后的修正
| 位置 | 问题 | 修正 |
|---|---|---|
| `get_local_path.cpp` `AB_stop::concat_path` | AB 轨迹第一个点 (0,0,0) 拼上去后和分叉点**重合**，产生零长度线段，该处曲率被算成 0，前馈会跳一下 | 跳过 AB 轨迹中离原点 < 1cm 的点 |
| `get_local_path.cpp` 构造函数 | `path.csv` 录制结束时常多存一个相同点（当前文件最后两行就是重复的），同样产生零长度线段 | 读入后去掉连续重复点（相距 < 1cm），日志打印去掉了几个 |
| `get_local_path.cpp` `AB_stop::load` | 只录一边时，读不存在的另一个文件会打印“无法打开CSV文件”，看起来像出错 | 先用 `std::filesystem::exists` 判断，不存在就直接走镜像 |
| `get_local_path.cpp` `ab_select_callback` | 运行中切换 A/B，新路径变短时 `last_index` 可能越界 | 重建后把 `path_transformer.last_index` 限制在新路径范围内 |
| `save_ab_path.cpp` | Ctrl+C 时最后一段（< `save_dx`）没存，停车点会比实际少最多 10cm | 析构时补存最后位置 |
| `save_ab_path.cpp` | 参数错误时在构造函数里 `rclcpp::shutdown()`，之后 main 里又 `spin` + `shutdown` | 改为构造失败标记 `ok_=false`，main 里判断后再 `spin` |
| `save_ab_path.cpp` | `M_PI` 在部分编译器下未定义 | 改用本文件常量 `AB_PI` |

已验证：`save_ab_path.cpp` 用桩头文件通过 `g++ -fsyntax-only -Wall -Wextra`；`AB_stop` + `CSV_get_Path` 抽出来单独编译运行，确认只录 A 时 B 自动镜像、按分叉点朝向旋转后落在正确一侧、分叉点不重复、AB 段速度 = 1.5。完整节点仍需上车 `colcon build`。

## 已知限制（未改，供参考）
- AB 段的朝向基准用的是分叉点那一个点的 `yaw`（录制时的里程计 yaw），单点有噪声时 AB 段整体会有小角度偏差；实测偏的话可用 `concat_dy` 微调横向，或改为用分叉点前几个点的连线方向。
- `slow_point`（当前 15 点 ≈ 1.5m）若比 AB 段长，减速会从分叉点之前就开始，属正常。
