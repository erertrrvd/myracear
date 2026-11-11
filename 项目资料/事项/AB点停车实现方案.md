# AB 点停车实现方案（录两遍路径 + 起跑指令选 A/B）

## 1. 方案概述

- 提前录两遍路径：第一遍终点停在 A 区中心，存成 `path_A.csv`；第二遍终点停在 B 区中心，存成 `path_B.csv`。
- 比赛起跑的启动指令里带一个参数 `park:=A` 或 `park:=B`，`get_local_path` 加载对应的 csv。
- 车沿着该路径跑，走到路径末尾自动减速、停车，所以停在录制时的终点位置。
- 不需要拼接路径，不需要 `/park_target` 话题，不需要拆分路径。

## 2. 规则依据（比赛规则 PDF）

- 第 3 页：第一圈现场建图，第二圈基于地图导航，第二圈按停车检测区提示停进对应区域；第一圈不需要停车。
- 第 5 页第 (8)–(10) 条：停车检测区中心随机放标志牌 A 或 B（线下赛由裁判现场随机给出）；停车区左 A 右 B（以运行方向为参考）；第 (9) 条写明第二圈“通过摄像头识别”标志牌。
- 第 8 页：三个及以上轮子停在对应区域内算成功，停车失败或未停车罚 20 秒；提前建图罚 50 秒。
- 第 9 页：停车识别不得超过 10 秒；任意位置停止超过 10 秒判任务失败；禁止 rosbag 预录/离线回放；人为干预每次罚 100 秒。

### 规则风险（务必赛前确认）

1. 本方案在起跑前人工指定 A/B，没有用摄像头识别标志牌，与第 (9) 条的字面要求不一致，可能被扣分。赛前建议向组委会/裁判确认：是否允许在发车前人工设定 A/B。
2. 预录路径可能被判“提前建图”（罚 50 秒），需要确认路径的使用方式是否合规。
3. 如果后续要求必须识别，只需增加摄像头识别节点，让它决定 `park` 的取值，两份 csv 的部分不用改。

## 3. 原理：靠路径终点停在 A/B

- `get_local_path` 把路径最后 `slow_point` 个点标成 v=-1（减速）、最后 `stop_point` 个点标成 v=-2（停车）；`pure_track_test` 走到 v=-2 的点就停车。
- A 路径的终点在 A 区中心，B 路径的终点在 B 区中心；A、B 车位左右相邻，终点只相差一个车位的横向距离。
- 停车精度 = 里程计定位精度 + 路径终点精度。规则只要求三轮入区，现场标定终点后余量通常够用。

## 4. 现有代码现状（同步自小车后）

- 路径文件写死：`src/get_local_path.cpp:1035` 的 `csv_path = "/home/racecar/src/my_racecar/include/path.csv"`，无法选择。
- `path.csv` 目前只有一份，168 个点，间隔约 0.1 米，type 列实际被读作速度 v，全为 1。
- `save_path.cpp` 把路径写到固定路径 `/home/racecar/src/my_racecar/include/path.csv`，所以录完一遍要手动改名保存。
- `ab_flag` 在 `get_local_path.cpp:1060` 声明后未使用，`concat_path` 已被注释掉。
- `launch/final_start.launch.py` 里的 `ab_flag`、`a_angular_z1` 等属于旧方案（`get_flag` + `move_control` 固定角度），与本方案无关。

## 5. 操作步骤

### 5.1 录两遍路径（小车上）

录 A：

```bash
ros2 launch my_racecar save_path.launch.py
# 手动遥控开完整路线，最后停在 A 区中心，然后 Ctrl+C
cp /home/racecar/src/my_racecar/include/path.csv /home/racecar/src/my_racecar/include/path_A.csv
cp /home/racecar/src/my_racecar/include/init_scan.csv /home/racecar/src/my_racecar/include/init_scan_A.csv
```

录 B（重新录一遍，终点停在 B 区中心）：

```bash
ros2 launch my_racecar save_path.launch.py
# 手动遥控开完整路线，最后停在 B 区中心，然后 Ctrl+C
cp /home/racecar/src/my_racecar/include/path.csv /home/racecar/src/my_racecar/include/path_B.csv
cp /home/racecar/src/my_racecar/include/init_scan.csv /home/racecar/src/my_racecar/include/init_scan_B.csv
```

注意：
- `save_path` 每次启动都会覆盖 `path.csv`，**同时也会覆盖 `init_scan.csv`**（它在收到第一帧 `/scan` 时保存起点雷达帧，见 `src/save_path.cpp` 的 `scan_callback`）。所以两个文件都要在录下一遍之前 `cp` 保存。
- 两遍录制和比赛发车，都必须从**同一个起点、同一个朝向**出发。路径坐标是以里程计启动时的位置为原点记录的，起点偏 1° 跑 17 米，终点就会横向偏约 0.3 米，A/B 车位相邻，偏太多就会停错车位。
- 录完用 `wc -l` 和 `tail -3` 检查两个文件，确认终点坐标确实一个在 A 区、一个在 B 区（横向相差约一个车位宽）。

### 5.2 代码改动（`src/get_local_path.cpp`，未应用，仅记录）

增加成员变量（紧跟 `csv_path` 定义，约 1035 行）：

```cpp
    std::string park_ = "none";   // "A" / "B"；"none" 时用默认 path.csv
```

构造函数中，在 `declare_parameter("start_icp", ...)` 之后：

```cpp
        this->declare_parameter("park", park_);
```

在 `get_parameter("start_icp", ...)` 之后（必须在 `CSV_get_Path path_loader(csv_path);` 之前，现有代码顺序满足）：

```cpp
        this->get_parameter("park", park_);
        if (park_ == "a") park_ = "A";   // 容忍小写
        if (park_ == "b") park_ = "B";
        if (park_ == "A" || park_ == "B")
        {
            csv_path = "/home/racecar/src/my_racecar/include/path_" + park_ + ".csv";
            // ICP 参考帧也要跟着路径走（见 6.2）
            init_scan_path_ = "/home/racecar/src/my_racecar/include/init_scan_" + park_ + ".csv";
        }
        else if (park_ != "none")
        {
            RCLCPP_FATAL(this->get_logger(), "park 只能是 A / B / none，收到: %s", park_.c_str());
            throw std::runtime_error("invalid park");
        }
        RCLCPP_INFO(this->get_logger(), "park=%s path csv: %s", park_.c_str(), csv_path.c_str());
```

在 `global_path_ = path_loader.get_path();` 之后，把原来只打印错误的空路径判断改成直接退出：

```cpp
        if (global_path_.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "路径为空或文件不存在: %s", csv_path.c_str());
            throw std::runtime_error("empty path");
        }
```

原因：`odom_callback` 里没有空路径保护，`find_nearest_index` 用 `path.size()-1` 做循环上界，路径为空时会下溢，越界访问后节点崩溃或行为异常。`park:=A` 但 `path_A.csv` 没拷到车上，就会触发这个问题，必须在启动时就报错停下。

没有传 `park` 时行为与现在完全一致，仍然读 `path.csv` 和 `init_scan.csv`。

### 5.3 launch 配置（`launch/move_test.launch.py`）

该文件已经 import 了 `DeclareLaunchArgument` 和 `LaunchConfiguration`，只需要额外 import `ParameterValue`：

```python
from launch_ros.parameter_descriptions import ParameterValue
```

在 `DeclareLaunchArgument('start_odom', ...)` 旁增加：

```python
        DeclareLaunchArgument('park', default_value='none',
                              description='停车点 A 或 B；none 时使用默认 path.csv'),
```

`get_local_path` 节点的 parameters 里增加一项：

```python
            parameters=[{
                'front_dist': 0.5,
                'pub_global_path': 0,
                'stop_point': 3,
                'slow_point': 15,
                'start_icp': 0,
                'park': ParameterValue(LaunchConfiguration('park'), value_type=str),
            }]
```

说明：
- 用 `ParameterValue(..., value_type=str)` 强制按字符串传。launch_ros 会把替换结果当 YAML 解析，空字符串可能被解析成空值导致启动失败，所以默认值用 `none` 而不是 `''`。
- `pure_track_test` 的下标参数也要按 A/B 分开，见 6.1。

### 5.4 比赛时起跑指令

```bash
# 停 A
ros2 launch my_racecar move_test.launch.py park:=A
# 停 B
ros2 launch my_racecar move_test.launch.py park:=B
```

启动后先看日志里的 `park=A path csv: .../path_A.csv`，确认加载的是对的文件再发车。

### 5.5 改完后重新编译

`path_*.csv` 是运行时读取的文件，不用重新编译；改了 `.cpp` 才需要在小车上编译（`colcon build --packages-select my_racecar`）。

## 6. 查漏补缺（对照现有代码发现的问题）

### 6.1 【必须改】`pure_track_test` 的下标参数要按 A/B 分开

`move_test.launch.py` 里 `pure_track_test` 用的是**路径点下标**来触发动作：

```python
'traffic_light_index': 101,
'first_stop_index': 80,
'second_stop_index': 229,
```

A、B 是两次独立录制，同一个位置在两份 csv 里的下标不同（车速、走线稍有差别，点数就会差几个到几十个）。只切换 csv、不切换这些下标，B 路径的红绿灯/人行道停车点就会提前或滞后。

另外，现有 `path.csv` 只有 168 个点，`second_stop_index: 229` 永远不会触发，说明这组参数和当前路径本来就没对上，录完 A/B 后需要分别重新标定。

做法：在 launch 里按 `park` 选两组参数，用 `OpaqueFunction` 读取启动参数：

```python
from launch.actions import DeclareLaunchArgument, OpaqueFunction

PARK_PARAMS = {
    'A':    {'traffic_light_index': 101, 'first_stop_index': 80, 'second_stop_index': 229},
    'B':    {'traffic_light_index': 101, 'first_stop_index': 80, 'second_stop_index': 229},
    'none': {'traffic_light_index': 101, 'first_stop_index': 80, 'second_stop_index': 229},
}   # A、B 的数值录完路径后分别标定

def make_pure_track(context):
    park = LaunchConfiguration('park').perform(context).upper()
    if park == 'NONE':
        park = 'none'
    params = {
        'linear_x': 1.0, 'd_min': 0.5, 'k_v': 0.5, 'kp_yaw': 1.0, 'kd_yaw': 0.0, 'ff_kp': 1.0,
        'theta_min': -25.0, 'theta_max': 25.0, 'f_index': 12,
        'traffic_light_time': 5.2, 'v_mode': 0, 'slow_v': 1.0,
    }
    params.update(PARK_PARAMS[park])
    return [Node(package='my_racecar', executable='pure_track_test',
                 name='pure_track_test', output='screen', parameters=[params])]

# LaunchDescription 里把原来的 pure_track_test Node 换成：
#     OpaqueFunction(function=make_pure_track),
```

标定方法：用 `park:=A` 跑一遍，看 `pure_track_test` 日志里打印的 `current_index`，记下车到人行道、红绿灯位置时的下标，填进 `PARK_PARAMS['A']`；B 同理。

### 6.2 【启用 ICP 时必须】起点雷达帧要和路径配对

`save_path` 每次录制都会覆盖 `init_scan.csv`，它是 ICP 校正起点位姿用的参考帧（`get_local_path` 的 `start_icp`、`init_scan_path_`）。如果只保留最后一次录的 `init_scan.csv`，A 路径就会用 B 的参考帧校正。

5.1 已加上拷贝 `init_scan_A.csv` / `init_scan_B.csv`，5.2 已让 `init_scan_path_` 跟着 `park` 切换。当前 `move_test.launch.py` 是 `start_icp: 0`，暂时不影响；但起点一致性是这个方案最大的误差来源，建议实跑时打开 ICP。

### 6.3 【建议改】末段减速其实没有减速

`move_test.launch.py` 里 `slow_v: 1.0`，和 `linear_x: 1.0` 一样，所以最后 `slow_point: 15` 个点（约 1.5 米）不会真正减速。停车点是 v=-2 的前 `stop_point: 3` 个点（约 0.3 米），车以 1 m/s 冲过去再刹车，停车位置受刹车距离影响，误差会变大。

建议：`slow_v` 降到 0.3~0.5，再根据实测调 `stop_point`，让车停稳后正好在车位中心。

### 6.4 【确认】最近点搜索不会跳到别的路段

`find_nearest_index`（`src/get_local_path.cpp:516`）从上一次的下标 `last_index` 往前找，只会单调向前，不会因为两圈轨迹重叠或终点靠近起点而跳到路径末尾提前停车。所以一条 csv 里包含两圈是可以的，这点不用改。

### 6.5 【确认】其他启动入口

`launch/move_new.launch.py` 也启动了 `get_local_path`（用的是旧的 `ab_flag`、`concat_dy` 参数，代码里已无效）。比赛只用 `move_test.launch.py` 的话不用管；如果用 `move_new.launch.py`，要按 5.3 同样改。

### 6.6 【确认】csv 会进 git

`.gitignore` 明确没有忽略 `*.csv`，`path_A.csv`、`path_B.csv`、`init_scan_A.csv`、`init_scan_B.csv` 可以直接提交，同步方法和之前一样。

## 7. 注意事项

- A、B 两份路径是独立录的，前半段不必完全一致，只要终点在对应车位内即可。
- 每次改动路径或终点，都要用 5.4 的指令实跑验证，不要只看文件。
- `path_A.csv`、`path_B.csv` 应提交到 git，并同步到小车和电脑，避免只存在一边。

## 8. 验证

- 启动日志确认 `park=A path csv: .../path_A.csv`；故意传 `park:=C`、或删掉 `path_A.csv` 再启动，确认节点直接报错退出。
- 分别用 `park:=A`、`park:=B` 各跑 10 次，记录是否三轮入区、终点横向误差。
- 两条路径各跑一次，确认人行道、红绿灯停车位置都对（验证 6.1 的下标）。
- 终点误差大时：降低末段速度（`slow_v`）、调整 `stop_point` / `slow_point`，检查 `start_icp` 重定位，必要时重新录终点。
- 确认没有传 `park` 时，仍然读 `path.csv` 且行为不变。
