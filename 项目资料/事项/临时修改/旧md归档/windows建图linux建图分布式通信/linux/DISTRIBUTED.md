# 电脑 CPU 分布式建图（车端发数据，电脑端 SLAM）

车端继续运行原硬件驱动和固定路径行驶控制，只增开里程计 TF；雷达 `/scan` 经 WiFi 由电脑端
ROS 2 接收，SLAM Toolbox 在电脑上建图，地图保存在电脑本地。此模式**不与**
`mapping_only.launch.py` 或 `race_mapping.launch.py` 同时运行。

ROS 2 不使用 ROS 1 的主从机设置；跨机通信由 DDS 直接完成，前提是两端网络互通。

## 网络与时间前提

- 电脑与车载机接入同一可互通局域网，先互相 `ping` 确认。
- 两端 ROS_DOMAIN_ID 相同（示例用 `0`），`ROS_LOCALHOST_ONLY` 均未设为 `1`，防火墙放行
  ROS 2 的发现与数据端口。发现失败时用 `ros2 multicast send/receive` 排查组播。
- **两端时钟必须一致**，否则 TF 查询失败，RViz 和 slam_toolbox 会刷
  `Message Filter dropping message ... queue is full`，扫描全部被丢弃。
  车载机时间若无法修改，把电脑时间对齐到车载机：`sudo timedatectl set-ntp false`，
  然后 `sudo date -s @<车载机时间戳>`（车载机上执行 `date +%s` 查看，或直接
  `sudo date -s @$(ssh root@192.168.5.100 date +%s)`），建图结束
  `sudo timedatectl set-ntp true` 恢复。

## 车端（SSH 到 root@192.168.5.100）

窗口一：日常硬件驱动（保持运行）。`Run_car.sh` 已启动雷达、IMU、编码器和
`base_footprint → base_link → laser_link` 静态 TF，不要再单独启动雷达驱动。

```bash
cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
bash Run_car.sh
```

窗口二：硬件驱动运行后，增开里程计 TF。原行驶入口若已启动 `imu_encoder_mix`，
必须 `start_odom:=false`：

```bash
cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
ros2 launch my_racecar distributed_mapping_vehicle.launch.py start_odom:=false
```

该入口只按需要启动 `imu_encoder_mix` 和 `odom_tf_adapter`（发布
`odom → base_footprint`），默认不重复发布雷达静态 TF。首次部署或代码更新后需先：

```bash
cd /home/racecar
source /opt/ros/humble/setup.bash
colcon build --packages-select my_racecar --symlink-install
```

## 电脑端（Ubuntu 22.04 原生 ROS 2 Humble）

电脑已改为 Ubuntu 系统，直接使用本机 ROS 2 Humble（需已安装 slam_toolbox、nav2_map_server
和 RViz2）。电脑端工作空间就是本工程所在目录 `/data/code/智能车/智能车(1)/my_racecar（复件）(1)`（包源码在其下的 `my_racecar（复件）/`，不再使用 `~/ros2_ws`）。若代码更新过，先重新编译：

```bash
cd "/data/code/智能车/智能车(1)/my_racecar（复件）(1)"
source /opt/ros/humble/setup.bash
colcon build --packages-select my_racecar
```

启动电脑端 SLAM（RViz 随该入口自动打开）：

```bash
source "/data/code/智能车/智能车(1)/my_racecar（复件）(1)/install/local_setup.bash"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch my_racecar distributed_mapping_pc.launch.py map_dir:=~/maps/distributed
```

`ROS_DOMAIN_ID` 要与车端实际值一致（两端都不设置时同为默认值 0）。RViz 中 Fixed Frame 选
`map`，添加 `/map`（Map）、`/scan_pc`（LaserScan）和 TF；雷达不显示时检查 LaserScan 的 QoS
Reliability 是否为 Best Effort。

电脑端 SLAM 不使用车端 `/tf`：车端 TF 经 WiFi 到达后与扫描时间对不上，slam_toolbox 会以
`earlier than all the data in the transform cache` 丢弃全部扫描。入口内的 `scan_tf_bridge`
把 `/scan` 转发为 `/scan_pc`（帧 `laser_pc`），并用每帧扫描自身的时间戳发布
`odom_pc → base_pc`（位姿取 `/encoder_imu_odom`）。里程计不可靠时加 `use_odom:=false`，
位姿恒为原点，完全靠激光扫描匹配建图（推车要慢，转向尤其要慢）。

## 保存地图

电脑端 SLAM 保持运行，另开终端执行（`run-index` 与启动时的 `run_index` 一致）：

```bash
source "/data/code/智能车/智能车(1)/my_racecar（复件）(1)/install/local_setup.bash"
ros2 run my_racecar save_mapping --run-index 1 --map-dir ~/maps/distributed
```

保存会生成 `~/maps/distributed/run1/`：`complete.json`、`map.posegraph`、`map.data`、
`map.pgm`、`map.yaml`、`map.raw`、`run.jsonl`。若 `run1` 已存在会拒绝覆盖；下一次试验启动和
保存都用新的 `map_dir`（如 `~/maps/distributed_02`）。

## 注意

- 电脑端只订阅数据、发布地图，**不会向车辆发送任何运动指令**。
- 电脑掉线会中断建图，但不会影响车上已运行的独立行驶流程。
- WiFi 丢包会直接降低建图质量（`/scan` 走 UDP best_effort）；先确认
  `ros2 topic hz /scan` 稳定再推车。
- 该流程尚未完成跨机实测；首次运行先低速或架空轮验证。
