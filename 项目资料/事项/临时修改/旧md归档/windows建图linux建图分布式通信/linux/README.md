# Linux 分布式建图（小车发数据，Ubuntu 电脑 SLAM）— 已实车跑通

> **按顺序复制就能用的完整指令，见上一级目录的 `README.md`（“完整运行指令”一节）。** 本文件偏重原理、文件对照和排查记录。

## 分工
```
小车 Linux/ARM64                         Ubuntu 22.04 电脑
Run_car.sh：雷达 /scan、IMU、编码器       scan_tf_bridge：/scan → 重采样 720 点 → /scan_pc
imu_encoder_mix → /encoder_imu_odom                         按每帧扫描时间戳发 odom_pc→base_pc
（odom_tf_adapter 已不再启动）           slam_toolbox：订阅 /scan_pc → /map、map→odom_pc
固定路径、行驶控制仍在车上        ─DDS/WiFi→  rviz2 查看；save_mapping 保存地图
```
电脑端**不使用车端 `/tf`**（时间戳对不上扫描、时钟有偏差会让 slam_toolbox 丢光扫描），只用 `scan_tf_bridge` 自己生成的 TF。电脑不向车发任何运动指令。

## 文件（备份位置 → 项目实际位置）
| 备份 | 项目位置 | 在哪台机器 | 说明 |
|---|---|---|---|
| `vehicle/distributed_mapping_vehicle.launch.py` | `mapping/launch/` | **小车** | **已改**：只启动 `imu_encoder_mix`，不再启动 `odom_tf_adapter`（电脑端不用车端 TF，省约 14% CPU） |
| `vehicle/odom_tf_adapter` | `mapping/scripts/` | 小车（**分布式建图已不用**） | `/encoder_imu_odom` → TF `odom→base_footprint`；仍被 `mapping_only` / `race_mapping`（车上跑 SLAM）使用，所以文件保留 |
| `pc/mapping/launch/distributed_mapping_pc.launch.py` | `mapping/launch/` | 电脑 | 电脑端入口：`scan_tf_bridge` + `slam_toolbox` + `mapping_status_logger` + rviz2 |
| `pc/mapping/scripts/scan_tf_bridge` | `mapping/scripts/` | 电脑 | **关键修复**：重采样固定点数 + 按扫描时间戳发 TF |
| `pc/mapping/scripts/mapping_status_logger` | `mapping/scripts/` | 电脑 | 记录 scans/odom/map_cells 到 `.run1-live.jsonl` |
| `pc/mapping/scripts/save_mapping` | `mapping/scripts/` | 电脑 | 保存 `map.pgm/yaml/raw/posegraph/data` |
| `pc/mapping/config/mapper.yaml` | `mapping/config/` | 电脑 | SLAM 参数（max_laser_range 12、occupancy_threshold 0.35 等） |
| `pc/mapping/rviz/slam_mapping.rviz` | `mapping/rviz/` | 电脑 | rviz2 配置（订阅 `/scan_pc`） |
| `DISTRIBUTED.md` | `mapping/DISTRIBUTED.md` | — | 项目里的原说明 |

这些是 git 提交 `c587ccd`（"分布式建图：电脑端 scan_tf_bridge 修复 SLAM 不出图…"）及之前 `d160d20` 引入/修改的内容。包的 `CMakeLists.txt` 需把这些脚本放进 `install(PROGRAMS ...)`（项目里已有）。

## 运行指令（每个代码块自带完整开头，复制即用，按顺序，可重复）
`ROS_DOMAIN_ID` 两端必须一致 = **42**（`DISTRIBUTED.md` 里写的 0 是示例）。地图目录每次按时间新建，不会重名。更完整的步骤和排错表见上一级目录 `README.md`。

### 一、小车（SSH：`ssh root@192.168.5.100`）
**窗口一（保持运行）**：
```bash
cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
bash Run_car.sh
```
**窗口二（保持运行）**：启动里程计 `imu_encoder_mix`（发 `/encoder_imu_odom`）；已经在运行就不重复启动。**已不再启动 `odom_tf_adapter`**（电脑端自己生成 TF，不用车端 `odom→base_footprint`）：
```bash
cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
if ros2 node list 2>/dev/null | grep -q imu_encoder_mix; then
  echo "imu_encoder_mix（/encoder_imu_odom）已经在运行，不重复启动（如需重启，先去原窗口 Ctrl+C）"
else
  ros2 launch my_racecar distributed_mapping_vehicle.launch.py
fi
```
**首次部署或代码更新后先编译**：
```bash
cd /home/racecar
source /opt/ros/humble/setup.bash
colcon build --packages-select my_racecar --symlink-install
source /home/racecar/install/local_setup.bash
```
**报 `found more than once`**（install 里有悬空符号链接）：
```bash
cd /home/racecar
source /opt/ros/humble/setup.bash
find /home/racecar/install/my_racecar/share/my_racecar -xtype l
find /home/racecar/install/my_racecar/share/my_racecar -xtype l -delete
```

### 二、Ubuntu 电脑
工作空间：`/data/code/智能车/智能车(1)/my_racecar（复件）(1)`。

**编译（代码更新过才需要）**：
```bash
cd "/data/code/智能车/智能车(1)/my_racecar（复件）(1)"
source /opt/ros/humble/setup.bash
colcon build --packages-select my_racecar
```
**联机检查**：
```bash
cd "/data/code/智能车/智能车(1)/my_racecar（复件）(1)"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
timeout 6 ros2 topic hz /scan
timeout 6 ros2 topic hz /encoder_imu_odom
ros2 topic info /scan -v | head -n 12
```
`/scan` 应稳定约 10 Hz 再推车；电脑端日志里不应再刷 `Message Filter dropping message`、`LaserRangeScan contains … expected …`。

**开始建图（SLAM + rviz2，窗口保持运行）**：
```bash
cd "/data/code/智能车/智能车(1)/my_racecar（复件）(1)"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
MAP_DIR=~/maps/distributed_$(date +%Y%m%d_%H%M%S)
echo "地图目录: $MAP_DIR"
ros2 launch my_racecar distributed_mapping_pc.launch.py map_dir:=$MAP_DIR
```
里程计不可靠时在最后一行末尾加 ` use_odom:=false`（纯激光匹配，推车要慢，转向更慢）。RViz：Fixed Frame 选 `map`，添加 `/map`、`/scan_pc`、TF；雷达不显示时把 LaserScan 的 Reliability 设为 Best Effort。

### 三、保存地图（SLAM 保持运行，另开终端；自动取最新目录）
```bash
cd "/data/code/智能车/智能车(1)/my_racecar（复件）(1)"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
MAP_DIR=$(ls -dt ~/maps/distributed_* | head -n 1)
echo "保存到: $MAP_DIR"
ros2 run my_racecar save_mapping --run-index 1 --map-dir $MAP_DIR
ls -l $MAP_DIR/run1
```
生成 `run1/`：`complete.json map.posegraph map.data map.pgm map.yaml map.raw run.jsonl`。`run1` 已存在会拒绝覆盖，下一次重新跑建图步骤（会新建目录）。第 2、3 趟把 `--run-index` 和启动时的 `run_index:=` 都改成 2 / 3，`MAP_DIR` 用同一个目录。

## 2026-10-05 同步的改动（Linux 电脑也要更新）
`pc/mapping/` 里的 `scan_tf_bridge`、`mapping_status_logger`、`map_autosave`（新增）、`mapper.yaml`、`distributed_mapping_pc.launch.py` 都有更新；`CMakeLists.txt` 的 `install(PROGRAMS …)` 要包含 `mapping/scripts/map_autosave`（项目里已加）。同步到 Linux 电脑后重新 `colcon build --packages-select my_racecar`。新增 launch 参数：`autosave_period`（默认 60 秒，0 关）、`beams`（默认 720）。自动保存目录为 `<map_dir>/autosave/`。
小车端：`vehicle/Run_car_safe.sh`、`vehicle/distributed_mapping_vehicle.launch.py`（不再启动 `odom_tf_adapter`），安装步骤见 `../../上传到小车/README.md`。

## 建图过程中解决的问题（摘自项目记录 3.5~3.6）
1. 点云能显示但 `map_cells=0` → 车端 TF 与扫描时间对不上 → `scan_tf_bridge` 自己发 TF。
2. 雷达每帧点数 366/367/368 → slam_toolbox 拒收 → 桥接重采样为固定 720 点（按实际点数推角度，同角度格取最近点，用"反向取样"避免大量 `inf`）。
3. 墙外放射状条纹 → `inf` 太多把射线擦穿墙 → 反向取样 + `max_laser_range: 12`。
4. 走动的人留残影 → `occupancy_threshold: 0.35`、`min_pass_through: 4`。
5. `Run_car.sh` 与新入口各启动一个 `imu_encoder_mix` → `/encoder_imu_odom` 交替跳变 → `start_odom:=false`。
6. 对时测不准/时钟被改 → 桥接不依赖时钟对齐。
