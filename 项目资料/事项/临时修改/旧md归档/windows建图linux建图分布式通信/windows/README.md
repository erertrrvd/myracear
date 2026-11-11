# 在 Windows 上做分布式建图（电脑 SLAM，小车发数据）

> **按顺序复制就能用的完整指令，见上一级目录的 `README.md`（“完整运行指令”一节）。** 本文件偏重原理、文件对照和排查记录。

## 结论
**Windows 能建图。** 之前"Windows 建不出图"的现象，根子不在 Windows：项目记录（3.4~3.5）里 Linux 上也遇到过同样的"数据到了、地图为空"，原因是
1. 车端 `/tf` 经 WiFi 后时间戳对不上扫描，slam_toolbox 把所有扫描丢掉；
2. 雷达每帧点数不固定（366/367/368），slam_toolbox 拒收点数变化的帧；
3. 无回波射线穿墙，把墙外擦成空闲。

这三条后来都在代码里修了（`scan_tf_bridge` + `mapper.yaml`），与操作系统无关。当时的 Windows 方案只"空载启动过"，没有用修复后的代码重试，所以不能算 Windows 失败。

## 本次验证（2026-10-04，Windows，`D:\ubuntu\slam` 的 pixi 环境）
用模拟小车（10 m × 3 m 走廊、每帧 366/367/368 点随机、0.25 m/s）喂数据，走完整条链路：`scan_tf_bridge` → `slam_toolbox` → `/map` → `save_mapping`。
- `/map` 生成：200×61 格、0.05 m，12261 格，占据 77 + 空闲 1682（与记录里 Linux 模拟测试的 12261 格一致）。
- 日志无 `earlier than all the data in the transform cache`、`queue is full`、`LaserRangeScan contains ... expected` 等报错。
- `save_mapping` 成功保存 `run1`：`map.pgm / map.yaml / map.raw / map.posegraph / map.data / complete.json / run.jsonl`。

**没有验证的**（需要实车）：车上的 `/scan`、`/encoder_imu_odom` 能否经 WiFi 被 Windows 的 DDS 收到；真实地图质量。这是网络问题，不是 SLAM 问题，见下面"联机"。

## 文件
| 备份位置（本目录下） | 实际位置 | 说明 |
|---|---|---|
| `mapping/launch/distributed_mapping_pc_windows.launch.py` | 项目 `mapping\launch\` | **新增**。Windows 专用 launch：不依赖编译安装 `my_racecar`（原 `distributed_mapping_pc.launch.py` 用 `Node(package='my_racecar')`，Windows 环境里没有这个包），直接用 pixi 的 python 跑 `mapping/scripts` 里的脚本。SLAM 逻辑和参数（`mapper.yaml`）与 Linux 版相同 |
| `mapping/scripts/scan_tf_bridge`、`mapping_status_logger`、`save_mapping` | 项目 `mapping\scripts\` | **原有，未修改**（与 Linux 同一份），放在这里是为了本文件夹自包含 |
| `mapping/config/mapper.yaml`、`mapping/rviz/slam_mapping.rviz` | 项目 `mapping\config\`、`mapping\rviz\` | **原有，未修改** |
| `D_ubuntu_slam/run_slam_windows.ps1` / `.bat` | `D:\ubuntu\slam\` | **新增**。一键：建图 / 联机检查 / 保存 |
| `D_ubuntu_slam/pixi.toml`、`fastdds_wifi_only.xml` | `D:\ubuntu\slam\` | **原有**。pixi 环境定义（ROS 2 Humble、slam_toolbox、rviz2）；DDS 只走 WiFi 网卡的配置（IP 现为 192.168.5.121，要与电脑 WiFi 的 IP 一致） |
| `fake_car_模拟小车.py` | （测试用，未放进项目） | 模拟小车，用来在没车时验证整条链路 |

没有改动任何已有代码。小车端和 Linux 完全一样，见 `../linux/README.md`。

## 运行（每个代码块自带完整开头，复制即用，按顺序，可重复）
小车端和 Linux 完全一样（`Run_car.sh` + `distributed_mapping_vehicle.launch.py`），**完整的小车端命令和排错表见上一级目录 `README.md` 的"完整运行指令"**。`ROS_DOMAIN_ID=42`，两端必须一致。**电脑必须先连上小车 WiFi（192.168.5.x）。**

### 1. 联机检查（约 8 秒出结果）
```powershell
cd D:\ubuntu\slam
.\run_slam_windows.bat -Check
```
通过标准：发现小车节点；`/scan`、`/encoder_imu_odom` 都"收到：是"；`/map 发布者: 0`。没连小车 WiFi 会红字提示并列出当前网卡。

### 2. 建图 + rviz2（窗口保持运行；地图目录自动为 `maps\run_<日期_时分>`）
```powershell
cd D:\ubuntu\slam
.\run_slam_windows.bat
```
没有里程计用 `.\run_slam_windows.bat -NoOdom`；不开 rviz 加 `-NoRviz`。

### 3. 保存地图（建图窗口保持运行，另开一个 PowerShell；自动存到最新目录）
```powershell
cd D:\ubuntu\slam
.\run_slam_windows.bat -Save
```
看到"第 1 趟建图已完整保存"才算成功，再关建图窗口。第 2、3 趟：
```powershell
cd D:\ubuntu\slam
$d = (Get-ChildItem D:\ubuntu\slam\maps -Directory | Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName
.\run_slam_windows.bat -RunIndex 2 -MapDir $d
```
保存同理加 `-Save`。其他参数：`-Domain 42`、`-CarIp 192.168.5.100`、`-AllNics`（不限定网卡，一般不要用）。

### 4. 保存后叠加 CSV 路径查看（离线）
```powershell
cd D:\ubuntu\slam
$m = (Get-ChildItem D:\ubuntu\slam\maps -Recurse -Filter map.yaml | Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName
.\view_csv.bat -Map $m -Extra "D:\debug\a.csv"
```

**网卡（重要，2026-10-04 实车排查结论）**：电脑有多块网卡（WLAN、手机 USB 共享、Meta 代理隧道 198.18.0.1、两块 VMware 虚拟网卡）时，DDS 发现流量走错网卡，表现为 rviz 里**没有点云、Map 宽高为 0**，slam_toolbox 一帧都没收到。实测：默认配置下发现不了小车任何节点；限定只走 WLAN（192.168.5.124）后立刻发现小车全部节点，收到 `/scan`（979 点）和 `/encoder_imu_odom`。脚本现在**默认**找出和小车同网段的网卡 IP，自动生成 `D:\ubuntu\slam\fastdds_car_nic.xml` 并启用，WiFi IP 变了也不用手改。旧的 `fastdds_wifi_only.xml` 写死 192.168.5.121，已不用。

## 实车排查记录：rviz 没有点云、Map 宽高为 0（2026-10-04）

**现象**：运行 `run_slam_windows.ps1` 后，节点都正常启动（`scan_tf_bridge`、`slam_toolbox`、`rviz2` 无报错），但 rviz 里没有雷达点云，Map 显示 `Width 0 / Height 0`，只有画面中心一个小坐标轴。

**先说结论**：SLAM 这边是正常的，问题是**电脑根本没收到小车的雷达数据**，卡在小车 → 电脑的 DDS 通信上（与之前 `-Check` 一直没输出是同一个问题）。

**从现象判断**
- 节点都起来了、没有报错：`scan_tf_bridge`、`slam_toolbox`、`rviz2` 本身没问题。
- 画面中心的小坐标轴是 `base_pc` / `laser_pc`：这是桥接节点自己发的静态 TF，说明 rviz 和 TF 链路是通的。
- Map 宽高为 0：slam_toolbox 要收到第一帧扫描才会生成地图，宽高为 0 就是一帧都没收到。
- LaserScan 没有点：同一个原因。

**排查（只读，没有改动任何系统设置）**
| 检查项 | 结果 |
|---|---|
| 网卡 | 5 块：WLAN（192.168.5.124）、以太网 2、Meta 代理隧道（198.18.0.1）、VMware VMnet1 / VMnet8 两块虚拟网卡 |
| 到小车的路由 | 走 WLAN，`ping` 小车通 |
| 防火墙 | Domain / Private / Public 三种配置文件都是关闭的 → **不是防火墙** |
| 默认配置下 DDS 发现 | 域 42：**发现不了小车任何节点**；域 0：也什么都没有 |
| 只走 WLAN 网卡（限定 `192.168.5.124`）| 域 42：**立刻发现小车 14 个节点**（`lslidar_driver_node`、`imu_encoder_mix` 等），收到 `/scan`（979 点）和 `/encoder_imu_odom` |
| 旧配置 `fastdds_wifi_only.xml` | 写死的 IP 是 192.168.5.121，与当前 WLAN IP（.124）不一致，加了也不生效 |

**根因**：电脑上多余的网卡（代理隧道、手机共享、VMware 虚拟网卡）干扰了 DDS 的发现流量，发现包没走到 WiFi 网卡上。

**修复**：`D:\ubuntu\slam\run_slam_windows.ps1` 每次启动都会自动查出"通往小车的那块网卡"的 IP（最初按路由查，后改为按"与小车同网段"查，见下方"后续补充"），生成只走这块网卡的 Fast DDS 配置 `fastdds_car_nic.xml` 并启用，WiFi IP 变了也不用手改。启动时会先打印：
```
DDS 只走网卡 WLAN (192.168.5.124)，小车 192.168.5.100，ROS_DOMAIN_ID=42
```
同时 `-Check` 改成约 8 秒内出结果（不再一直卡住），会直接报告：发现了哪些小车节点、`/scan` 与 `/encoder_imu_odom` 是否收到、`/map` 是否已有别的发布者（冲突）。参数 `-WifiOnly` 已去掉，新增 `-AllNics`（不限定网卡，一般不要用）。

**修复后的联机检查（实车，只检查数据能否收到）**
```
发现的节点: IMU_publisher、imu_encoder_mix、lslidar_driver_node、racecar_driver ... 共 14 个
/scan 发布者: 1  收到一帧: 是，979 点          (桥接节点会重采样成 720 点)
/encoder_imu_odom 发布者: 1  收到: 是
/map 发布者: 0                                   (建图前应为 0)
```

**没有验证的**：只确认了数据能收到，没有在实车数据上跑完整的建图（避免在不知情时往小车网络里发 `/map`）。出图后若效果不对，需要看 rviz 截图再判断。

**建议**：建图前把 Meta 代理关掉更稳妥（有网卡限定后不关也应该能用）。

**后续补充（同日）**：之后某次运行 `-Check` 选中了 `Meta (198.18.0.1)` 并显示 "DDS 没通"。原因是电脑当时连的是**别的 WiFi**（WLAN 为 10.105.129.143/24），不在小车网段，到 192.168.5.100 的流量被默认路由送进了 Meta 代理隧道。因此脚本的选卡逻辑已改为"找和小车同一网段的网卡"，找不到就红字提示没连小车 WiFi 并退出，不再用路由判断。完整的按顺序复制即用的指令见上一级目录 `README.md` 的“完整运行指令”一节。

## 2026-10-05 改进汇总（模拟验证，实车待验证）
- `map_autosave`（新增）：建图时每 60 秒自动存到 `<地图目录>\autosave\`，忘了 `-Save` 不会全丢。
- `scan_tf_bridge`：开了里程计却收不到 `/encoder_imu_odom` 时报错。实测没有里程计时位姿恒为原点，而 slam_toolbox 要移动超过 `minimum_travel_distance`(0.2 m) 才处理新扫描，所以地图几乎不增长（13:14 那趟就是这样，障碍只有 25 格）。
- `mapping_status_logger`：状态日志加 `known_cells`，地图 30 秒不增长写 `stalled` 事件。
- `run_slam_windows.ps1`：前台每 15 秒打印进度和告警（Python 节点的告警在 launch 输出里不会实时显示，所以改为读状态日志）；退出时检查有没有正式保存；`-Check` 增加重复节点和 `/scan`、`/encoder_imu_odom` 频率检测；新增 `-AutoSave`。
- `mapper.yaml`：`occupancy_threshold 0.35→0.20`，`min_pass_through 4→3`。依据见 `模拟测试/`：车速 1.5 m/s、带扫描畸变时，原参数只找回 2/12 个细锥桶，0.20/3 找回 6/12 且走动物体不留痕迹，0.10/2 开始留痕迹。车速 0.5 m/s 时各参数都能找回 6/6。重采样点数 `beams`（720→1080）和去畸变 `deskew` 在实验里没有稳定收益，默认值没改。
- `view_csv.ps1 -Inflate N`：只影响显示，障碍格加粗，小锥桶更醒目。
- 小车端：`Run_car_safe.sh`、不再启动 `odom_tf_adapter`，见 `../../上传到小车/README.md`。
- 以上结论都来自模拟（`模拟测试/` 里的脚本可复现）；真实雷达的时间戳约定、噪声、遮挡与模拟不同，**实车上要再建一张图对比**。

## 联机常见问题（Windows 特有，需实车确认）
1. **收不到小车话题**：`-Check` 里"发现的节点：无"。依次查：电脑连的是小车 WiFi；两端 `ROS_DOMAIN_ID` 相同（实测小车是 42，域 0 什么都发现不了）；不要加 `-AllNics`；防火墙（本机三种配置文件当前都是关闭的）；只 `ping` 通不代表 DDS 通。
2. **时钟**：`scan_tf_bridge` 用每帧扫描自己的时间戳发 TF，不依赖两端时钟一致，所以时钟偏差不影响建图。
3. **旧文档过期**：事项里的 `windowsslam建图指令.md` 还是旧入口（`launch\distributed_mapping_pc.launch.py`、直接订 `/scan`、域号 0），没有 `scan_tf_bridge`，按它做会再次遇到"不出图"。以本文为准。
4. 已有的 Linux 版入口 `distributed_mapping_pc.launch.py` 在 Windows 上不要用（找不到 `my_racecar` 包）。

## 附：模拟测试怎么复现（没有小车时验证整条链路）
三个 PowerShell 窗口，每个都自带完整开头；`<windows目录>` 换成本文件夹的实际路径。

窗口一（SLAM，地图目录自动按时间命名，不会重名）：
```powershell
cd D:\ubuntu\slam
$env:ROS_LOCALHOST_ONLY="1"; $env:ROS_DOMAIN_ID="77"
$t = Get-Date -Format "yyyyMMdd_HHmmss"
pixi run --frozen ros2 launch "C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\launch\distributed_mapping_pc_windows.launch.py" map_dir:=D:/ubuntu/slam/maps_test_$t rviz:=false
```
窗口二（模拟小车，约 24 秒）：
```powershell
cd D:\ubuntu\slam
$env:ROS_LOCALHOST_ONLY="1"; $env:ROS_DOMAIN_ID="77"
pixi run --frozen python "<windows目录>\fake_car_模拟小车.py" 24
```
窗口三（建图跑起来后，保存测试地图；自动取最新的 maps_test_*）：
```powershell
cd D:\ubuntu\slam
$env:ROS_LOCALHOST_ONLY="1"; $env:ROS_DOMAIN_ID="77"
$d = (Get-ChildItem D:\ubuntu\slam -Directory -Filter "maps_test_*" | Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName
pixi run --frozen python "C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\scripts\save_mapping" --run-index 1 --map-dir ($d -replace '\\','/')
```



恢复小车rviz
cd /home/racecar
for B in /home/racecar/_bak_rviz/*; do
  F=$(basename "$B" | tr '_' '/')
  echo "备份 $B  ->  应恢复到 $F"
done