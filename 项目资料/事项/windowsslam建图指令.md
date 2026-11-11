# Windows 电脑运行 SLAM 建图指令

本文记录 `my_racecar` 的**分布式建图**操作。小车的 Linux 车载机运行雷达、编码器与 IMU 驱动、原固定路线行驶控制，并提供建图所需的 TF；Windows 电脑接收 `/scan` 与 TF，用电脑 CPU 运行 SLAM Toolbox，地图保存在 `D:\ubuntu\slam\maps`。

这套入口已在电脑本机验证可启动，但**尚未用当前车载网络完成跨机建图实测**。首次使用时请先按下文检查实时话题和 TF。ROS 2 使用 DDS 通信，不需要设置 ROS 1 的 `ROS_MASTER_URI` 或所谓“主从机 IP”。

## 一、代码和文件放在哪里

| 位置 | 作用 |
| --- | --- |
| `C:\Users\lzc2\Desktop\my_racecar（复件）\launch\distributed_mapping_pc.launch.py` | 电脑端入口，运行 SLAM Toolbox 和状态日志；不运行行驶控制 |
| `C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\config\mapper.yaml` | 电脑端 SLAM 参数；配置 `/scan`、`odom`、`base_footprint` 和 `map` |
| `C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\scripts\save_mapping` | 电脑端保存地图脚本 |
| `D:\ubuntu\slam` | Windows 原生 ROS 2 Humble、SLAM Toolbox 的 Pixi 环境和缓存 |
| `D:\ubuntu\slam\maps` | 电脑上的地图目录；成功保存第一趟后使用 `run1` 子目录 |
| `/home/racecar/src/my_racecar/launch/distributed_mapping_vehicle.launch.py` | 小车端入口，只补建图里程计 TF；需要部署到车上并构建 |
| `/home/racecar` | 小车 ROS 2 工作空间；硬件驱动和固定路线控制仍在车上运行 |

**不要把整个 Windows 工作副本、Windows 编译产物或 Pixi 环境复制到小车。**电脑端直接使用 C 盘源码中的 Python launch、参数文件与保存脚本；小车端只部署其所需的源码文件，并在 Linux 车载机上构建。D 盘文档目录是说明文档所在位置，并非上面命令实际读取的源码目录。

## 二、首次使用前：确认车上已有新入口

先在**小车 SSH 终端**检查：

```bash
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
test -f /home/racecar/src/my_racecar/launch/distributed_mapping_vehicle.launch.py && echo '车端入口已存在'
ros2 pkg executables my_racecar | grep odom_tf_adapter
```

若第一条未显示“车端入口已存在”，在**电脑 Windows PowerShell** 且已连上小车网络时只传这个 launch 文件：

```powershell
scp 'C:\Users\lzc2\Desktop\my_racecar（复件）\launch\distributed_mapping_vehicle.launch.py' root@192.168.5.100:/home/racecar/src/my_racecar/launch/
```

随后在**小车 SSH 终端**构建并重新加载工作空间：

```bash
cd /home/racecar
source /opt/ros/humble/setup.bash
colcon build --packages-select my_racecar --symlink-install
source /home/racecar/install/local_setup.bash
ros2 launch my_racecar distributed_mapping_vehicle.launch.py --show-args
```

如果 `odom_tf_adapter` 可执行文件本身不存在，说明车上还缺少与该入口对应的脚本/包安装内容，不能只传一个 launch 文件就直接运行；需要先核对车载源码与 C 盘工作副本的差异，再部署适合车载 Linux 的源文件并构建。不要直接覆盖已有车载驱动和路径文件。

## 三、每次建图时的启动顺序

### 第 1 步：小车继续按原流程启动

在小车上先启动已验证的雷达、编码器、IMU 驱动与固定路线控制。这里不改动 `path.csv`、雷达驱动或小车速度控制。开始行驶前，确保 `/scan`、`/encoder_imu_odom` 正在实时发布，不能仅凭 `ros2 topic list` 中出现名字判断有数据。

若使用原 `move_test.launch.py` 默认配置，它已启动 `imu_encoder_mix`，下面的 `start_odom` 应设为 `false`。

### 第 2 步：小车另开 SSH 终端，启动建图 TF

**下面的命令在小车 Linux SSH 终端执行，不在 Windows PowerShell 执行：**

```bash
cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
ros2 launch my_racecar distributed_mapping_vehicle.launch.py start_odom:=false
```

如果原行驶入口**确实没有**启动 `imu_encoder_mix`，才把参数改成 `start_odom:=true`。若原车载流程已经发布同一条 `odom → base_footprint` TF，则不要再启动这个新车端入口，以免出现两个 TF 发布者。车载雷达已有 `base_footprint → base_link → laser_link` 链时也不要另加重复静态 TF。

### 第 3 步：电脑 PowerShell 启动 SLAM

电脑需要连入能与 `192.168.5.100` 通信的小车局域网。使用普通 **Windows PowerShell**，不需要在小车 SSH 中输入这段，也不用每次重新安装 Pixi 包：

```powershell
$pixi = "$env:LOCALAPPDATA\pixi\bin\pixi.exe"
$manifest = 'D:\ubuntu\slam\pixi.toml'
$env:PIXI_CACHE_DIR = 'D:\ubuntu\slam\cache'
$env:ROS_DOMAIN_ID = '0'
$env:ROS_LOCALHOST_ONLY = '0'

& $pixi run --manifest-path $manifest ros2 launch 'C:\Users\lzc2\Desktop\my_racecar（复件）\launch\distributed_mapping_pc.launch.py' map_dir:=D:/ubuntu/slam/maps
```

`ROS_DOMAIN_ID=0` 是示例值。电脑和正在运行的车载 ROS 2 节点必须处于同一域；若车上设置了其他值，应把电脑端改为车上的值。两端还要能通过 DDS 发现并传输数据；只 `ping` 通不证明 ROS 2 通信已通。电脑端启动后，保持此窗口运行。小车移动且建图输入正常时，电脑端 SLAM 会持续更新 `/map`。

## 四、首次联机检查：建图是否真的有反应

**车上 Linux SSH 终端：**

```bash
ros2 topic info /scan -v
ros2 topic info /encoder_imu_odom -v
ros2 node list
```

**电脑另开 PowerShell 窗口：**新窗口需要重新设置 Pixi 路径和 ROS 域，然后检查是否收到雷达数据。

```powershell
$pixi = "$env:LOCALAPPDATA\pixi\bin\pixi.exe"
$manifest = 'D:\ubuntu\slam\pixi.toml'
$env:PIXI_CACHE_DIR = 'D:\ubuntu\slam\cache'
$env:ROS_DOMAIN_ID = '0'
$env:ROS_LOCALHOST_ONLY = '0'

& $pixi run --manifest-path $manifest ros2 topic info /scan -v
& $pixi run --manifest-path $manifest ros2 topic echo /scan --once --qos-reliability best_effort
& $pixi run --manifest-path $manifest ros2 run tf2_ros tf2_echo odom laser_link
```

最后一条 TF 命令会持续输出，看到有效变换后按 `Ctrl+C`。然后再检查地图话题与本次状态日志：

```powershell
& $pixi run --manifest-path $manifest ros2 topic info /map -v
Get-Content -LiteralPath 'D:\ubuntu\slam\maps\.run1-live.jsonl' -Tail 3
```

日志文件可能保留上一次本机空载测试记录。确认末尾的**时间戳是本次运行**，并在小车移动时观察 `scans`、`odometry_messages`、`map_cells` 是否增长。电脑看得到话题名称，但没有发布者或收不到消息，仍不能算联机成功；`/scan` 能收到但地图不增长时，再查 TF 连通、车和电脑的时钟、雷达时间戳及 QoS。

可选：电脑另开 PowerShell 启动 RViz。保持与建图窗口相同的环境变量，然后运行：

```powershell
& $pixi run --manifest-path $manifest rviz2
```

RViz 的 Fixed Frame 设为 `map`，添加 `/map` 的 Map、`/scan` 的 LaserScan 和 TF。若雷达图层不显示，检查 LaserScan 的 Reliability 是否设为 `Best Effort`。RViz 只负责查看，不负责建图或控制车辆。

## 五、建图完成后在电脑保存地图

**电脑端 SLAM 窗口仍须保持运行。**另开 Windows PowerShell 窗口执行：

```powershell
$pixi = "$env:LOCALAPPDATA\pixi\bin\pixi.exe"
$manifest = 'D:\ubuntu\slam\pixi.toml'
$env:PIXI_CACHE_DIR = 'D:\ubuntu\slam\cache'
$env:ROS_DOMAIN_ID = '0'
$env:ROS_LOCALHOST_ONLY = '0'

& $pixi run --manifest-path $manifest python 'C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\scripts\save_mapping' --run-index 1 --map-dir D:/ubuntu/slam/maps

Get-ChildItem -LiteralPath 'D:\ubuntu\slam\maps\run1'
```

保存成功时应检查 `run1` 下是否有 `map.pgm`、`map.yaml`、`map.raw`、`map.posegraph`、`map.data`、`complete.json` 和本趟日志。脚本对已存在的 `run1` 拒绝覆盖；再做一次独立试验时，使用新的 `map_dir`，避免误覆盖原结果。保存失败或文件缺失时不能仅凭 RViz 能显示 `/map` 认为文件已落盘。

## 六、冲突边界与当前验证状态

- **不要同时运行**电脑端分布式入口、车载 `mapping_only.launch.py`、车载 `race_mapping.launch.py` 或其他 SLAM Toolbox；同一时间只让一个节点发布 `/map` 与 `map → odom`。
- 同一时间只保留一个 `imu_encoder_mix` 和一个 `odom → base_footprint` TF 发布者。`move_test.launch.py` 默认启动 `imu_encoder_mix`，所以本步骤通常使用 `start_odom:=false`。
- 若小车原启动脚本还运行 AMCL、Nav2 或其他地图定位节点，先核对其是否发布 `/map` 或 `map → odom`；重复发布会使 TF/地图不稳定，不能简单称为“绝对不冲突”。
- 电脑端 launch **不发 `/cmd_vel` 或 `/car_cmd_vel`**，小车原固定路线控制仍在车上。电脑掉线会中断建图；是否影响小车继续行驶，取决于车载原控制流程本身，不由此电脑入口保证。
- 电脑运行 SLAM 仍会占用车端雷达、里程计和网络资源；比赛前应检查车端控制周期、CPU、网络和地图连续性。
- 截至本文整理，本机已确认 Windows Pixi、ROS 2 和 SLAM Toolbox 存在，电脑端曾空载启动；**跨机收发扫描、移动建图、实车保存结果尚需按上文现场验证**。

更详细的设计和诊断说明见 C 盘源码工作副本中的 `mapping/DISTRIBUTED.md`；原行驶命令及旧建图模式见同目录的 `运行命令.md`。本文件的指令只对应“车载行驶、Windows 电脑建图”这一模式。
