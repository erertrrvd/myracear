# 本车雷达话题接入 SLAM 的过程与迁移提醒

本文按本次对话和车载机终端日志整理。目标是让本车真实雷达 `/scan` 进入独立建图流程，并为以后移植到其他小车留下可复用的排查顺序。用户最后反馈“现在可以运行了”；对话尚无最终地图文件、RViz 截图或地图精度验收输出。因此，“能启动并运行”不等于“地图已验收可比赛”。

## 一、从照片到独立建图入口

车辆侧面照片显示雷达位于车体上层，但没有标尺，不能单凭照片确定 `base_footprint` 的机械位置、雷达扫描中心或扫描 +X 与车头方向。C 盘 Git 工作副本新增 `launch/mapping_only.launch.py`，默认试运行外参为 `base_footprint → laser_link`：`x=0、y=0、z=0.30 m、roll=0、pitch=0、yaw=0`。`0.30 m` 是目测高度，水平重合和朝向也是待核对假设，不是测量结果。

独立入口启动 `imu_encoder_mix`、`odom_tf_adapter`、`mapping_status_logger`、SLAM Toolbox 和 `mapping_ready`，默认还发布雷达静态 TF。它不读取 `path.csv`，不启动 `move_test.launch.py` 或固定路线控制，也不主动发布 `/cmd_vel`。地图目录是 `/home/racecar/maps/mapping_only`，与比赛三趟地图分开。

## 二、代码同步与第一次启动错误

最初在电脑上 `git push origin main` 遭到 `fetch first`：车端 Git 远端有本地尚未合入的提交。先合入远端再推送，没有强制推送。车端 `/home/racecar/src/my_racecar` 有未提交的 `launch/move_test.launch.py` 修改，`git pull --rebase` 因工作区改动而拒绝；改为 `git fetch origin`、`git merge origin/main`，日志显示快进并保留了未提交改动。车上 `colcon build --packages-select my_racecar --symlink-install` 成功，只能证明编译完成，不能证明雷达链路已通。

第一次启动时，`mapping_ready`、`mapping_status_logger`、`odom_tf_adapter` 均报 `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`。日志显示脚本实际由 Conda Python 3.9 执行，而车载 ROS 2 Humble 的 `rclpy` 是系统 Python 3.10 扩展。将这三个脚本及 `save_mapping` 的 shebang 改为 `/usr/bin/python3`，同步并重新编译后，导入错误消失，节点能启动。这是 Python 解释器不匹配，和雷达是否发布扫描无关。

Windows PowerShell 中的 `ping -c 3` 也曾报错；`-c` 是 Linux 风格参数，在 Windows 可用 `ping 192.168.5.100`。MobaXterm 曾显示 `Network error: Software caused connection abort`，它表示该 SSH 会话断开，不能由此推断车载 ROS 进程是否仍在运行。

## 三、节点启动了却没有地图：从话题名查到发布者

修复 Python 后，日志显示 SLAM Toolbox 和静态 TF 等进程已经启动，但 `mapping_ready` 超时。`/home/racecar/maps/mapping_only/.run1-live.jsonl` 曾连续记录 `scans=0、odometry_messages=0、map_cells=0`。当时 `ros2 topic list` 虽列出 `/scan`，`ros2 topic info /scan -v` 却显示 **Publisher count: 0**，只有 `mapping_status_logger`、`slam_toolbox`、`mapping_start_check` 三个订阅者；`/encoder`、`/IMU_data` 也各为 **Publisher count: 0**。所以当时上游传感器没有在当前 ROS 图中发布。**话题名出现不等于有数据，更不能证明 SLAM 已建图。**

车载 `/home/racecar/src` 中存在 `lslidar_driver`、`hipnuc_imu`、`encoder`、`racecar_driver`，`ros2 pkg list` 和 `ros2 pkg executables` 也可见对应包和程序。但包已安装不代表进程已启动。车上的 `/home/racecar/src/README.md` 给出了单独雷达驱动命令 `ros2 launch lslidar_driver lslidar_launch.py`；随后找到了本车更完整的 `bash /home/racecar/Run_car.sh` 入口，本次使用它启动硬件，不再叠加单独雷达驱动。

`Run_car.sh` 的实际启动日志显示：`lslidar_driver_node`、IMU `talker/listener`、`encoder_node`、`racecar_driver_node`、RViz 和三个静态 TF 启动；雷达被识别为 **M10**，`/dev/laser` 以 `460800` 波特率打开，编码器 `/dev/encoder` 打开，IMU 配置为 `/dev/imu`、`115200`、话题 `/IMU_data`。这些只对本车此次运行成立，不应照搬到另一辆车。RViz 的 `Message Filter dropping message ... queue is full` 表示显示队列丢弃扫描，单凭它不能判定雷达故障或建图成功。

在硬件驱动运行期间，`ros2 topic echo /scan` 收到了真实 `sensor_msgs/msg/LaserScan`：`header.frame_id=laser_link`，`scan_time≈0.10 s`，距离数组有有效数值。个别 `inf` 表示对应方向无有效回波。至此可确认**该时刻**雷达发布成功。

## 四、车载 TF、超时和最终运行顺序

`Run_car.sh` 已发布下列 TF：

```text
base_footprint -- z=0.15 m --> base_link -- x=0.07 m --> laser_link
                                  └── x=0.1653 m --> IMU_link
```

本车 `laser_link` 的已有链条合成平移约为 `(0.07,0,0.15) m`。若独立建图入口同时发布默认 `base_footprint → laser_link`，会产生重复的雷达子帧 TF。因此本车在 `Run_car.sh` 一起运行时，建图命令要加 `publish_laser_tf:=false`。这意味着默认 `z=0.30 m` 的照片估值**并未参与本次实际运行**。现有 `0.07/0.15 m` 也只是车上原配置，不是尺量结论。

有一次约 06:04 收到 `/scan`，另一次约 06:08 启动建图后仍在 60 秒后报“等待 `/scan` 与 `/map`”并退出。两次并非同一时刻的同步检测；不能从 06:04 的扫描推断 06:08 期间驱动持续在线。车上旧版 `mapping_ready` 的这句还是初始提示，不能单凭它断言两个话题都不存在。后续的 `KeyboardInterrupt`、`rcl_shutdown already called`、`imu_encoder_mix: Owner died` 出现在关闭阶段，不能倒推为最先超时的根因。

本车采用以下顺序，并保持两个长期运行的 SSH 窗口打开：

```bash
# 窗口一：硬件驱动
cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
bash Run_car.sh
```

```bash
# 窗口二：独立建图
cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
ros2 launch my_racecar mapping_only.launch.py publish_laser_tf:=false ready_timeout:=180
```

`ready_timeout:=180` 只是延长首次排查等待时间，默认值为 60 秒；它不修复缺失数据，也不是地图质量参数。用户在排查后反馈“现在可以运行了”，但未提供最终成功日志。因此不能认定 180 秒配置是唯一修复原因，也不能断言前次超时仅因驱动窗口关闭。

建图运行**同时**在第三个窗口检查：

```bash
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
ros2 topic info /scan -v
ros2 topic info /map -v
timeout 8s ros2 topic echo /scan --once --qos-reliability best_effort
tail -n 3 /home/racecar/maps/mapping_only/.run1-live.jsonl
```

`/scan` 有发布者且能 `echo`，只证明雷达输入。日志 `scans` 持续增加才说明建图流程收到了扫描；`/map` 由本次 SLAM 发布且地图栅格持续更新，才说明地图链路工作。还要核对 `map → odom → base_footprint → base_link → laser_link` 与 RViz 显示。若 `/map` 来自旧 `map_server` 或另一套 SLAM，不要把它误认为本次建图结果；同一时刻也不能有两个 `map → odom` 发布者。

## 五、保存和实车验收仍需做

建图窗口继续运行，人工确认地图质量后保存：

```bash
ros2 run my_racecar save_mapping --run-index 1 --map-dir /home/racecar/maps/mapping_only
ls -l /home/racecar/maps/mapping_only/run1/
```

应核对 `complete.json`、`map.posegraph`、`map.data`、`map.pgm`、`map.yaml`。本对话没有这些文件的实际检查输出。首次实车验收还要尺量 `base_footprint` 到雷达扫描中心的前后、左右、高度，确认扫描 +X 与车头方向；静止检查扫描和墙面方向，缓慢直行和转弯检查地图连续性、墙面重影、里程计及时间戳。入口就绪检查通过不等于地图精度达到比赛要求。

## 六、以后映射到其他小车的检查顺序

1. 在目标车确认 ROS 发行版、工作空间、驱动入口、设备名和 `ros2 pkg executables`；不能从本车 `Run_car.sh`、`/dev/laser`、M10 型号推断另一辆车相同。
2. 先启动目标车驱动，再检查 `ros2 topic info /scan -v` 的**发布者数**，用 `ros2 topic echo /scan --once --qos-reliability best_effort` 查实际帧名、时间戳、距离和更新情况。仅看 `ros2 topic list` 不够。
3. 核对目标车 TF 的父子关系和每条边的发布者；驱动已发布车体到雷达 TF 时，关闭建图入口的重复 TF。没有发布时再使用针对目标车测量的外参。不能照搬本车 `0.07/0.15 m` 或照片初值 `0.30 m`。
4. 本项目入口还需要 `/encoder`、`/IMU_data` → `/encoder_imu_odom` → `odom → base_footprint`，不是纯雷达里程计。另一辆车若没有这条链，需实现适合其硬件的里程计或雷达里程计方案，单纯重映射 `/scan` 不够。
5. 只启动一套 SLAM，检查 `/map` 的实际发布者、`map → odom` 是否唯一、日志里扫描和地图是否持续更新。超时先查同一时刻的发布者、TF、时间戳、QoS、ROS 域和 DDS，再考虑算法参数。
6. 保存栅格地图和位姿图，观察车辆缓慢移动及转弯时地图是否连续、墙面是否重影，然后再评估路线和比赛精度。每辆车都需重新校准。

## 后续方案：电脑 CPU 建图、车载机继续固定路径

用户提出比赛时车载 CPU 需运行固定路径和校准，SLAM 运算改由 Windows 电脑承担。C 盘工作副本因此新增 `launch/distributed_mapping_vehicle.launch.py`（车端仅提供 `odom → base_footprint` TF）和 `launch/distributed_mapping_pc.launch.py`（电脑端启动 SLAM Toolbox 与状态日志），完整命令写在同目录《运行命令.md》和 C 盘的 `mapping/DISTRIBUTED.md`。车端原有雷达、融合里程计、固定路径和校准仍独立运行；分布式入口不订阅后改写或发布行驶指令。电脑计划保存栅格图和位姿图到 `D:\ubuntu\slam\maps`。

这不是 ROS 1 的 `ROS_MASTER_URI` 主从机设置。ROS 2 两端依靠 DDS 通信，电脑必须实际有能运行的 ROS 2 与 SLAM Toolbox。Windows 原生 RoboStack/Pixi 环境已安装在 `D:\ubuntu\slam\.pixi\envs\default`；本机验证了 `ros2` 命令、SLAM Toolbox 可执行文件和保存接口，短暂启动建图入口后在 `D:\ubuntu\slam\maps` 生成了状态日志。离线测试的扫描、里程计和地图计数均为 0。用户当前连接小车网络时不能同时上互联网，因此安装与车载联调分阶段进行。**新方案尚未部署到车、尚未跨机收发 `/scan`、尚未保存地图，不能认作实车验证通过。**
