# 分布式通信、交叉编译与 Windows / Linux 运行 ROS 2

> 本文记录这次智能车建图讨论的结论、实际完成的工作和仍需实车验证的部分。它解释运行原理；分布式建图的完整操作命令以 C 盘工作副本 `mapping/DISTRIBUTED.md` 和本目录《运行命令.md》为准。

## 一、先把三个问题分开

1. **在哪台机器编辑源码？** Windows、Linux 都可以。编辑器所在系统不决定程序最终运行在哪台机器。
2. **为哪个系统和 CPU 编译？** 运行在小车上的程序必须适配小车的 Linux / ARM64 环境；运行在本电脑上的程序必须适配本电脑的 Windows / x64 环境。同一份源码可以分别编译出两个平台的程序，但编译结果不能直接互换。
3. **实际在哪台机器执行？** 节点在哪里启动，计算就主要占用哪台机器的 CPU。跨机传话题只移动数据，不会自动把已在车上启动的算法转移到电脑。

ROS 2 是跨平台的软件框架，支持在 Linux 和 Windows 上编译、运行节点；其跨机话题通信通常通过 DDS 中间件实现。这不意味着所有第三方驱动、SDK、源码和设备接口也天然跨平台。Windows 上的 `.exe` / `.dll` 不能直接复制到 Linux 车载机运行；Linux 上编译的可执行程序及 `.so` 也不能直接当作 Windows 程序使用。

## 二、本车为什么把建图放在 Windows，硬件和行驶留在 Linux

小车已经在 Ubuntu 22.04 / ROS 2 Humble 环境中使用雷达、IMU、编码器和原固定路径控制。比赛时用户希望车载 CPU 优先运行固定路径及校准，由电脑承担 SLAM 计算。于是采用以下分工：

```text
小车 Linux / ARM64
  雷达驱动 → /scan
  编码器 + IMU → /encoder_imu_odom → odom → base_footprint
  车体到雷达的静态 TF：base_footprint → base_link → laser_link
  原固定路径、校准、行驶控制：仍在车上
                         │
                         │ ROS 2 DDS，局域网传扫描、里程计和 TF
                         ▼
电脑 Windows / x64
  SLAM Toolbox：订阅 /scan、里程计 TF、雷达 TF
  发布 /map 和 map → odom
  RViz：查看雷达、地图、TF
  保存地图：D:\ubuntu\slam\maps
```

车载入口 `launch/distributed_mapping_vehicle.launch.py` 只补建图所需的里程计 TF；若原行驶流程已经启动 `imu_encoder_mix`，应使用 `start_odom:=false`，避免重复融合节点。电脑入口 `launch/distributed_mapping_pc.launch.py` 启动 SLAM Toolbox 和状态日志，读取 `mapping/config/mapper.yaml`，不发布行驶命令。旧 `mapping_only.launch.py` 与 `race_mapping.launch.py` 是车上运行 SLAM 的入口，在分布式模式下不要同时启动，否则车上仍会承担 SLAM 运算，也可能出现重复 `/map` 或 `map → odom` 发布者。

车上仍会消耗 CPU 来采集传感器、计算里程计、执行控制，并增加一定网络发送开销；“电脑 CPU 建图”不等于“车上 CPU 零占用”。电脑断网或休眠会中断电脑端建图，原本独立运行的车载控制不会因这个建图入口自动停止。

## 三、Windows 为什么能运行 ROS 2、SLAM Toolbox 和 RViz

这里安装的是**为 Windows 编译的包**，并非把小车上的 Linux 二进制文件复制过来运行。RoboStack 的 ROS 2 Humble 包提供 Windows 64 位构建，Pixi 将包和依赖安装到隔离环境：

```text
D:\ubuntu\slam\pixi.toml                 环境清单
D:\ubuntu\slam\pixi.lock                 锁定的依赖版本
D:\ubuntu\slam\.pixi\envs\default      实际 Windows ROS 2 环境
D:\ubuntu\slam\cache                     安装包缓存
D:\ubuntu\slam\maps                      地图和建图日志目录
```

环境中已安装 ROS 2 Humble、SLAM Toolbox 与 RViz2。本机已验证 `ros2` 命令、`async_slam_toolbox_node.exe`、SLAM Toolbox 的 Python 保存接口以及 `rviz2.exe --help`；曾短暂启动电脑端建图入口，SLAM 节点正常启动，状态日志写入 `D:\ubuntu\slam\maps\.run1-live.jsonl`。当时没有接入小车，日志的扫描、里程计、地图计数均为 0。**这证明电脑端程序能够启动，不证明两机通信或实车地图已经完成。** RViz 的实际图形渲染和实车显示也尚未验收。

启动电脑端建图的 PowerShell 示例：

```powershell
$pixi = "$env:LOCALAPPDATA\pixi\bin\pixi.exe"
$manifest = 'D:\ubuntu\slam\pixi.toml'
$env:PIXI_CACHE_DIR = 'D:\ubuntu\slam\cache'
$env:ROS_DOMAIN_ID = '0'       # 示例值，必须和车端一致
$env:ROS_LOCALHOST_ONLY = '0'
& $pixi run --manifest-path $manifest ros2 launch 'C:\Users\lzc2\Desktop\my_racecar（复件）\launch\distributed_mapping_pc.launch.py' map_dir:=D:/ubuntu/slam/maps
```

在另一个设置了相同 ROS 环境变量的 PowerShell 窗口运行 RViz：

```powershell
& $pixi run --manifest-path $manifest rviz2
```

RViz 的 Fixed Frame 设为 `map`，添加订阅 `/map` 的 Map、订阅 `/scan` 的 LaserScan 和 TF 显示。雷达不显示时核对 LaserScan 的 Reliability 是否应设为 Best Effort。RViz 只负责显示，建图算法仍由电脑端 SLAM Toolbox 执行。

建图成功且电脑端 SLAM 仍在运行时，用同一个 Pixi 环境调用项目保存工具；保存目录须与启动时相同：

```powershell
& $pixi run --manifest-path $manifest python 'C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\scripts\save_mapping' --run-index 1 --map-dir D:/ubuntu/slam/maps
```

应在 `D:\ubuntu\slam\maps\run1` 检查 `complete.json`、`map.pgm`、`map.yaml`、`map.posegraph`、`map.data` 等实际文件。仅有 `.run1-live.jsonl` 状态日志不代表已经生成地图。

## 四、ROS 2 分布式通信不是 ROS 1 的“主从机”

ROS 2 不靠设置 `ROS_MASTER_URI` 来指定一台主机。两端节点通过 DDS 发现并交换话题、服务及 TF。至少需要核对：

- 两端 ROS 2 域号相同，例如都为 `ROS_DOMAIN_ID=0`；`ROS_LOCALHOST_ONLY=0`，不能只允许本机通信。
- 两端 DDS 实现与网络配置可互通；Windows 防火墙和网络允许发现流量与双向 UDP。能 `ping` 或 SSH 到车上，只证明 IP 网络的部分功能，不能证明 DDS 已收到 `/scan`。
- 电脑确实收到小车实时发布的 `/scan`，并检查其 `header.frame_id`、时间戳、频率和 QoS。本车先前观察到雷达帧为 `laser_link`。
- TF 连通且各条 TF 只有一个发布者：`map → odom → base_footprint → base_link → laser_link`。扫描时间戳与 TF 时间范围要对应，两机时钟应校准。
- 电脑上的 `/map` 由本次 SLAM 发布，状态日志的 `scans` 和 `map_cells` 持续增长；再检查缓慢直行和转弯时的地图连续性与墙面重影。

若电脑同时接手机 USB 网络共享和小车 Wi-Fi，正常目标是**手机接口负责互联网，小车 Wi-Fi 负责访问小车**，不需要 ROS 1 式主从设置或 Windows 网络桥接。要注意 Windows 的 VPN/虚拟网卡可能改变路由或影响 DDS 选择网卡。曾有一次检查显示 USB 网络适配器已连接，但 WLAN 是 `Disconnected`；对 `192.168.5.100:22` 的测试却经过 `Meta Tunnel`，因此不能把该结果当作已经通过小车 Wi-Fi 实测。应在小车 Wi-Fi 确实连接后重新检查真实网卡、路由和 ROS 2 话题。手机 USB 网络共享相当于给电脑增加一个网络接口，并不把小车变成互联网设备。

## 五、为什么最初在 Windows 编辑、在小车上编译

小车运行 Linux / ARM64，而当前电脑是 Windows / x64。普通 Windows 本机编译通常产生 Windows 用的程序，不能直接部署到小车。最初的流程是：

```text
Windows 编写和修改源码
    → 把源码同步到小车
    → 小车使用自己的 ROS 2 Humble、编译器和硬件依赖编译
    → 在小车 Linux 上运行
```

这样最容易让构建时的 ROS 2 库、设备 SDK 和目标系统保持一致。它不表示 Windows 只能编辑源码，也不表示 Windows 不能编译 ROS 2 程序；只是 **Windows 本机编出的 Windows 程序** 与 **给车载 Linux 运行的程序** 是两种产物。

### 交叉编译是什么

交叉编译允许在电脑上完成编译，却生成小车 Linux / ARM64 可运行的程序。需要准备面向目标架构的编译器、与车上匹配的系统库和 ROS 2 库、全部第三方依赖及相应工具链配置，通常还需从车载系统取得匹配的 sysroot 或使用等价构建环境。产物依然是 Linux / ARM64 程序，而非 Windows `.exe`。完成后仍要部署到车上，验证动态库、设备、实时数据和控制行为。

交叉编译可以节省车上编译时间，但前期配置和维护成本更高。若车上能顺利完成 `colcon build`，直接在车上编译通常更省事；当车载 CPU 编译慢、构建频繁时，再考虑电脑交叉编译。电脑 Windows 中运行 SLAM Toolbox 属于**电脑本机运行 Windows 版程序**，不是交叉编译。

## 六、“代码兼容 Windows”到底指什么

Windows 能安装外部包并把它们链接、编译、组合运行。是否能把一个具体 ROS 2 项目搬过去，取决于各节点的依赖是否有 Windows 构建、源码是否调用 Linux 专有接口、硬件是否有 Windows 驱动，以及项目能否在 Windows 实际构建和连接设备。不能只看 `package.xml` 或目录名就宣布“兼容”。

逐项判断顺序：

1. ROS 2 发行版、CPU 架构和所需消息包是否匹配；自定义消息须在目标环境构建并保持接口一致。
2. 第三方库、SDK、Python 包是否有 Windows 版本；路径、编译器和链接方式是否正确。
3. 项目代码是否使用 `/dev/...`、`can0`、`<unistd.h>`、Linux 系统调用、`.so`、`/usr/local/lib` 等 Linux 专有内容。
4. 在 Windows 真正构建/导入、启动节点、连接设备或收到话题，并检查输出。通过静态代码阅读只得到可行性判断，不是运行验收。

### 本机两个项目的具体例子

**`C:\Users\lzc2\Desktop\UR5E机械臂(伺服电机)\jidianqi`：** 其 ROS 2 节点主要用 Python。`ur5e_controller/ur5e_controller/motor_node.py` 中通过 `pyserial` 读写串口，这个库可跨平台，但代码默认使用 `/dev/ttyUSB0`，备用为 `/dev/ttyACM0`。Windows 上通常需识别实际 `COM` 口，把端口改成参数，并接硬件检查波特率、协议帧和执行结果。视觉节点 `ur5e_vision/ur5e_vision/person_tracker.py` 直接使用 `pyrealsense2`、OpenCV、NumPy、Pillow 和模型文件；这些依赖、相机驱动以及模型路径也须在 Windows 分别验证。仅启动 `rviz2` 不能代替相机和电机节点。

**`D:\VSCode\ros2技术集\Autonomous (副本)`：** 它不是单个简单的 ROS 2 Python 包，包含 FAST-LIO2、Livox、RealSense、底盘、点云等工作空间。例如 `fastlio2/CMakeLists.txt` 查找 Livox 驱动、PCL、Eigen 和 OpenMP；当前 Livox 驱动构建文件查找 `/usr/local/lib` 下的 Linux `.so`；底盘代码包含 `<unistd.h>`，其配置使用 `can0`。这些可以逐项移植，但不能把 Linux `.so` 改个后缀当作 Windows DLL，也不能简单把 `can0` 改名为 `COM3`。需要找到或编译 Windows 版库，并按实际 USB-CAN 设备的 Windows 驱动/API 改造通信层，再逐节点编译与实测。

因此“能改”与“已能运行”不同。串口端口参数化通常工作量较小；Livox SDK、FAST-LIO2、底盘 CAN 及整套依赖的 Windows 移植工作量更大。也可以保持硬件驱动在 Linux，仅把计算或 RViz 放到 Windows，像本车分布式建图一样减少移植范围。

## 七、为什么很多实车项目仍选择 Linux

ROS 2 自身能跨平台，但具体传感器厂家驱动、底盘 SDK、设备接口和项目构建说明经常以 Ubuntu 为主要目标。本车原有硬件驱动及行驶程序也已经在车载 Linux 环境中使用。继续保留它们，可以复用现有设备接入和已跑通的控制流程；把算力密集且较容易跨平台的 SLAM/RViz 放在电脑，是当前任务中改动范围较小的方案。

这不是说 Windows 不能做机器人控制。若目标设备的 SDK、依赖及项目代码在 Windows 上均已验证，整套放在 Windows 运行完全可行；若项目依赖 Linux 设备接口，则 Linux 往往减少移植成本。最终应依据实际目标机、驱动、实时数据、CPU 负载和现场测试选择部署位置。

### 对“Windows 也能干，为什么仍要 Linux”的进一步说明

讨论中提出的概括很有用：**本次不直接接触车载硬件的建图和可视化任务可以放在 Windows 电脑；车上已有的设备驱动和控制依照其实际支持的系统留在 Linux。** 需要把“可以运行”和“当前整套系统已验证”区分开：Windows 与 Linux 都能执行 ROS 2 算法节点，但不同平台的包版本、DDS、性能和构建条件仍需分别验证，不能说在任何项目里都“完全等价”。

1. **纯计算与可视化可以离开小车。** 本方案的电脑端 SLAM Toolbox 只需经网络接收 `/scan` 和 TF，就可在电脑 CPU 上做扫描匹配和地图更新；RViz 同样通过 ROS 2 话题查看结果。它们不需要电脑直接连接小车雷达或电机。已经证实 Windows 节点能启动，但尚未证实车端话题可跨机到达并生成实车地图。
2. **硬件驱动要看“这台设备、这版 SDK、这个部署环境”。** 车载雷达、IMU、编码器和控制链目前在 Linux 上使用，保留它们可以避免重新移植及重新验收。`Autonomous (副本)` 的 Ranger 底盘确实使用 `can0`，但这不能推断本项目 `my_racecar` 的电机也走 `can0`；当前 C 盘工作副本没有提供这种证据。也不能笼统说“摄像头 SDK 都只有 Linux 版”：具体相机需查其型号和 SDK。Linux 设备名、Linux 库路径和厂商仅提供的 Linux 驱动，是迁移到 Windows 时必须解决的实际问题，而非操作系统理论上无法驱动硬件。
3. **Atlas 200I DK A2 的目标系统不能从电脑端 OS 自由替换。** 用户此前提供的车载登录信息显示其为基于 Ubuntu 22.04 的 ARM64 板卡。昇腾官方列出的 Atlas 200I A2 相关 CANN 目标系统是 Linux 系列；本车已部署的软件与工具链围绕该环境。Windows 电脑可以做编辑、网络通信和部分计算，但不等于能把当前车载系统直接改装成 Windows。是否使用 NPU、哪部分节点调用 CANN/ACL，仍应从本车实际代码与运行进程核对，不能只凭板卡型号断定本次建图使用了 NPU。
4. **项目生态影响迁移成本。** 本项目及一些传感器厂商给出的构建脚本和文档以 Ubuntu 为目标，例如 Livox ROS Driver 2 官方说明列出 Ubuntu 与对应 ROS 2 版本；遇到构建问题时沿用其文档更直接。ROS 2 本身官方支持 Windows，RoboStack 也提供了本次使用的 Windows 包。不能在没有统计证据时把 Windows 社区支持说成“少一个数量级”；对具体项目更有意义的是逐一查看所需包有没有 Windows 构建与维护记录。

因此，当前做法是按已经验证的硬件接入和计算任务来分工：**车载 Linux 接设备并执行原有控制；电脑 Windows 承担 SLAM 和 RViz。** 这是本项目减少移植量的选择，不是 Windows 永远只能做纯算法，也不是所有机器人硬件必须使用 Linux。若某一套设备有适用的 Windows 驱动和 SDK，且该项目经过 Windows 构建与实机测试，也可以改变分工。

## 八、当前完成情况和下一次验收

**已完成：** Windows Pixi 环境中的 ROS 2 Humble、SLAM Toolbox、RViz2 安装；电脑端建图节点启动检查；状态日志写入 D 盘；项目中车端 TF 入口与电脑端 SLAM 入口的源码拆分。项目环境清单为 `D:\ubuntu\slam\pixi.toml`，地图目标目录为 `D:\ubuntu\slam\maps`。

**仍未完成现场确认：** 新车端入口部署；电脑与小车之间的 DDS 发现和 `/scan`、TF 实时收发；实车移动时地图更新；RViz 显示效果；地图及位姿图完整保存；地图精度和车载控制周期验收。因此不能把“Windows 上节点能启动”说成“分布式建图已经在比赛车上成功”。

下次联机按顺序检查：小车 Wi-Fi 实际连接 → 电脑到车的真实路由 → 两端 ROS 域与 DDS → 电脑收到 `/scan` 和 TF → `slam_toolbox` 的 `/map` 和日志计数增长 → RViz 连续性 → D 盘地图文件完整保存 → 观察车载行驶 CPU、控制周期和网络占用。

## 参考资料

- [ROS 2 官方跨平台介绍](https://docs.ros.org/en/humble/_downloads/2a9c64e08982f3709e23d20e5dc9f294/ros2-brochure-ltr-web.pdf)
- [RoboStack 的 Windows 版 SLAM Toolbox 包](https://prefix.dev/channels/robostack-humble/packages/ros-humble-slam-toolbox)
- [Pixi / RoboStack 机器人环境说明](https://pixi.prefix.dev/v0.79.0/robotics/)
- [Livox ROS Driver 2 官方安装说明](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/README.md)
- [昇腾官方 CANN 软件包支持的操作系统](https://www.hiascend.com/doc_center/source/zh/CANNCommunityEdition/80RC1alpha003/softwareinst/instg/instg_0020.html)
- 本项目的 `C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\DISTRIBUTED.md` 和本目录《运行命令.md》记录实际使用命令。
