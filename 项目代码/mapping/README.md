# 雷达独立建图与固定路径比赛三趟建图

`mapping_only.launch.py` 用于先检查雷达、TF 和地图质量，不启动行驶控制，也不读取 `path.csv`。原 `race_mapping.launch.py` 仍用于固定路径比赛的三趟建图：它使用 `move_test.launch.py` 的 `get_local_path`、`pure_track_test` 参数和 `path.csv`，建图检查通过后会启动原控制。第 1 趟从空图开始，第 2、3 趟读取上一趟已完整保存的位姿图继续建图；每趟分别保存，不覆盖前一趟。

## 上车前检查

车载机需要 ROS 2 Humble、`encoder_imu`、`my_racecar`、`slam_toolbox` 和真实的 `/scan`、`/encoder`、`/IMU_data` 驱动。仅比赛三趟入口需要 `/home/racecar/src/my_racecar/include/path.csv` 和原控制使用的 ICP 参考扫描文件；独立建图入口不需要它们。

先在车上确认 `/scan` 的 `header.frame_id`。雷达驱动或其他启动文件如果已经发布车体到雷达的 TF，只保留一个 TF 发布者。建图入口独占发布 `odom -> base_footprint` 和 `map -> odom`；不要同时启动旧 `slam_mapping.launch.py`、仿真或其他 EKF/SLAM 入口。

在车载工作空间安装本包后加载该工作空间，例如：

```bash
colcon build --packages-select my_racecar
source install/local_setup.bash
```

车载机的 Conda `base` 环境使用 Python 3.9，而 ROS 2 Humble 的 `rclpy` 安装在系统 Python 3.10。建图目录下的四个可执行 Python 脚本固定使用 `/usr/bin/python3`；若终端出现 `rclpy._rclpy_pybind11` 缺失，先运行 `/usr/bin/python3 -c 'import rclpy; print("rclpy OK")'` 核对系统解释器，再重新编译并加载本包。不要通过安装另一个 Python 版本的 `rclpy` 来掩盖解释器不一致。

## 独立建图：先验证雷达和地图

新增入口默认发布 `base_footprint -> laser_link`，外参为 `x=0.00 m`、`y=0.00 m`、`z=0.30 m`、`roll=0`、`pitch=0`、`yaw=0`。车体照片只显示雷达位于上层，没有标尺，也无法确认车头方向、扫描原点或 `base_footprint` 的实物位置。**0.30 m 是目测高度初值；x/y 重合及雷达 +X 朝车头只是试运行假设，均非实测外参。**

车上只运行一个建图入口，确认没有其他节点发布相同的 TF 或控制话题。首次启动时保持底盘动力未使能，并准备从车载机观察：

```bash
ros2 topic info /scan -v
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 launch my_racecar mapping_only.launch.py
```

如果 `/scan.header.frame_id` 不是 `laser_link`，传入实际帧名，例如 `laser_frame:=laser`。测量后可用 `laser_x`、`laser_y`、`laser_z`（米）和 `laser_roll`、`laser_pitch`、`laser_yaw`（弧度）覆盖默认值，例如 `laser_x:=0.04 laser_y:=-0.02 laser_z:=0.27 laser_yaw:=0.05`；此处数字仅演示参数格式，不是本车测量结果。若底盘或雷达驱动已经发布可用的车体到雷达 TF，添加 `publish_laser_tf:=false`，同时确认 TF 链仍完整且没有重复发布。默认地图目录为 `/home/racecar/maps/mapping_only`，可用 `map_dir:=<另一独立目录>` 覆盖；若目录中已存在 `run1`，入口会拒绝覆盖。

独立入口启动 `imu_encoder_mix`、`odom_tf_adapter`、`mapping_status_logger`、静态 TF（默认启用）、SLAM Toolbox 和 `mapping_ready`，**不会启动 `move_test`、`get_local_path`、`pure_track_test`，也不发布 `/cmd_vel`**。就绪检查需要在默认 60 秒内连续收到有效扫描，看到足够的地图障碍且能查询完整 TF；超时会关闭本入口。检查通过只表示数据链初步可用，建图仍继续运行，不能作为地图精度验收。让车轮低速转动或由人缓慢推车时，先确认 `/encoder_imu_odom` 随运动更新，再观察地图是否连续。

在建图入口**仍运行**且已人工确认地图质量后，另开车载终端保存：

```bash
source install/local_setup.bash
ros2 run my_racecar save_mapping --run-index 1 --map-dir /home/racecar/maps/mapping_only
```

核对 `mapping_only/run1/complete.json`、`map.posegraph`、`map.data`、`map.pgm` 和 `map.yaml` 均已生成，再停止建图入口。要重新试建图，请为新试验指定新的 `map_dir`，保留已有结果供比较。

实车验收时核对 `/scan` 的帧名、频率和时间戳，并查询 `odom -> base_footprint -> 雷达帧` 与 `map -> odom`。在 RViz 中显示 `/scan`、`/map`、TF；静止时检查扫描与墙面方向，缓慢直行和转向时检查墙面是否重影、地图是否扭曲。然后实测车体参考点到雷达**扫描中心**的前后、左右、高度距离，以及扫描 +X 相对车头的方向，修正外参。转弯重影也可能来自里程计、扫描时间戳或回调延迟，不能只靠修改外参解决。

## 三趟运行

每一趟起跑前把车静止放在起点区域。第 2、3 趟的 `start_x/y/yaw` 是**上一趟地图坐标系中的起跑位姿**，默认零值只是起调值，必须现场核对。每次只运行一个 `race_mapping.launch.py`；旧 `move_test.launch.py` 由它在建图检查通过后自动启动，不能再另开一个。

第 1 趟：

```bash
ros2 launch my_racecar race_mapping.launch.py run_index:=1 map_dir:=/home/racecar/maps \
  laser_frame:=laser_link laser_x:=<实测值> laser_y:=<实测值> \
  laser_z:=<实测值> laser_yaw:=<实测值>
```

完成固定路径、车停稳后，**在建图 launch 保持运行时**另开终端：

```bash
source install/local_setup.bash
ros2 run my_racecar save_mapping --run-index 1 --map-dir /home/racecar/maps
```

看到完整保存提示，核对 `/home/racecar/maps/run1/complete.json` 后再停止 launch。第 2、3 趟依次把上面两个命令中的编号改为 `2`、`3`。新一趟启动前会校验上一趟的 `.posegraph`、`.data`、`.pgm`、`.yaml`、`map.raw`、`run.jsonl` 和 SHA-256；缺失或改变即拒绝启动。第 2、3 趟还会将扫描与上一趟保存的栅格快照匹配，检查通过才启动原路径控制。

现场可调启动参数：`start_x`、`start_y`、`start_yaw`、`start_radius`（默认 0.8 m）、`start_yaw_tolerance`（默认 0.5 rad）、`min_match_score`（默认 0.2）、`ready_timeout`（默认 60 s）。这些门限需要用正确放车和故意错放样本校准；如果错放也能通过，应提高门限并检查赛道几何。初次低速、架空轮验证前不要直接按比赛速度运行。

存图命令只在人工确认本趟已经结束后执行。`runN/complete.json` 中记录地图尺寸、障碍数、保存时间与文件校验；`runN/run.jsonl` 每五秒记录扫描、里程计、控制话题与地图数量。控制节点的详细终端输出和 ROS 日志也可另行归档。电脑掉线不影响车载建图；电脑不保存地图的唯一副本。

## 电脑实时看图

电脑与车载机接入同一可互通的局域网，两端使用 ROS 2 Humble 并设置相同的 `ROS_DOMAIN_ID`。检查 `ROS_LOCALHOST_ONLY` 未设为 `1`，尽量采用相同 DDS 实现；网络必须允许 ROS 2 的发现与数据传输。若发现失败，先在两端互相 `ping`，再用 `ros2 multicast send/receive` 和 `ros2 topic list` 排查网络或防火墙。

电脑只运行 RViz，Fixed Frame 选 `map`；显示 `/map`、`/scan` 和 TF。可直接加载本目录的 `rviz/slam_mapping.rviz`：

```bash
rviz2 -d '<电脑上的 my_racecar 路径>/mapping/rviz/slam_mapping.rviz'
```

车上查看 `ros2 topic hz /scan`、`ros2 topic echo /map --once`，电脑上查看 `ros2 topic list` 和 RViz。两台设备应同步时钟；若 RViz 报 TF extrapolation、消息过期，先核对时间、扫描时间戳与雷达坐标系。

## 电脑端 SLAM（WiFi 传雷达数据，电脑 CPU 建图）

车端只发数据（可继续行驶），电脑经 WiFi 接收后在本机运行 SLAM Toolbox。完整流程、网络与时间前提、电脑端（Ubuntu 原生 ROS 2 Humble）命令和地图保存方法见 **`DISTRIBUTED.md`**。该模式与 `mapping_only.launch.py` / `race_mapping.launch.py` 互斥，不能同时运行。

该模式下车端可继续按固定路径行驶，电脑掉线只中断建图，不影响车上独立行驶流程；WiFi 丢包会降低建图质量（`/scan` 走 UDP best_effort），先确认 `ros2 topic hz /scan` 稳定再验证。电脑端建出的地图在车上的三趟比赛流程中不能直接使用；比赛仍需按「三趟运行」一节在车上建图。

## 文件说明

- `DISTRIBUTED.md`：电脑 CPU 分布式建图的完整操作文档（车端/电脑端命令、保存、时钟处理）。
- `launch/distributed_mapping_vehicle.launch.py`：车端入口；只按需启动 `imu_encoder_mix` 和 `odom_tf_adapter`，默认不重复发布雷达静态 TF。
- `launch/distributed_mapping_pc.launch.py`：电脑端入口；本机运行 SLAM Toolbox、状态日志和 RViz，通过 WiFi 接收车上数据。
- `launch/mapping_only.launch.py`：近似外参和独立建图入口；不启动固定路径控制。
- `launch/race_mapping.launch.py`：三趟编号、旧图校验、建图起跑检查、原固定路径启动顺序。
- `config/mapper.yaml`：仅 SLAM Toolbox 的实车建图参数；扫描距离和匹配阈值仍需实车调节。
- `scripts/odom_tf_adapter`：把旧 `/encoder_imu_odom` 首帧重置为建图原点，只发布建图所需 TF。
- `scripts/mapping_ready`：在控制启动前检查地图、扫描、TF 和旧图吻合度。
- `scripts/mapping_status_logger`：记录本趟建图及控制话题活动。
- `scripts/save_mapping`：保存并校验本趟地图与位姿图。
- `scripts/scan_tf_bridge`：电脑端分布式建图用；把 `/scan` 重采样为固定点数转发到 `/scan_pc`，并按每帧扫描时间戳发布 `odom_pc → base_pc`，SLAM 不依赖车端 TF。
- `rviz/slam_mapping.rviz`：电脑查看实时地图的 RViz 配置。
