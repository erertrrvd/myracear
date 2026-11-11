# 分布式建图：Windows 与 Linux 两套实现

小车发雷达 `/scan` 和里程计 `/encoder_imu_odom`，电脑用 SLAM Toolbox 建图，地图存电脑。两套方案**逻辑完全相同**（同一个 `scan_tf_bridge` + `mapper.yaml`），区别只在电脑的操作系统和启动方式。

| | Linux 电脑 | Windows 电脑 |
|---|---|---|
| 状态 | **已实车跑通** | 模拟小车端到端跑通（出图+保存）；**未实车联机** |
| 环境 | Ubuntu 22.04 原生 ROS 2 Humble，`colcon build` 编译 `my_racecar` | `D:\ubuntu\slam` 的 pixi 环境（RoboStack ROS 2 Humble + slam_toolbox + rviz2），不编译包 |
| 入口 | `ros2 launch my_racecar distributed_mapping_pc.launch.py` | `D:\ubuntu\slam\run_slam_windows.ps1`（内部用 `distributed_mapping_pc_windows.launch.py`） |
| 小车端 | 相同 | 相同（见 `linux/README.md` 的"小车"部分） |
| 说明 | [`linux/README.md`](linux/README.md) | [`windows/README.md`](windows/README.md) |

## 目录
```
windows建图linux建图/
├─ README.md                       本文件
├─ linux/
│  ├─ README.md                    Linux 方案：文件、运行指令、遇到的问题
│  ├─ DISTRIBUTED.md               项目里的原说明
│  ├─ vehicle/                     小车端：distributed_mapping_vehicle.launch.py（只启动 imu_encoder_mix）、odom_tf_adapter（已不被它启动，仅作备份）
│  └─ pc/mapping/                  电脑端：launch、scripts、config、rviz
└─ windows/
   ├─ README.md                    Windows 方案：结论、验证过程、文件、运行指令、联机排错
   ├─ mapping/                     launch（Windows 专用，新增）+ scripts/config/rviz（原有，未改）
   ├─ D_ubuntu_slam/               run_slam_windows.ps1/.bat、pixi.toml、fastdds_wifi_only.xml（旧，已不用；新脚本自动生成 fastdds_car_nic.xml）
   └─ fake_car_模拟小车.py          无车时的端到端测试
```

## 完整运行指令（复制即用，按顺序，可重复）

**约定**
- 每个代码块**自己带齐开头**（`cd` + `source` / 环境变量），从头到尾整块复制粘贴就能跑，不依赖上一块留下的状态，所以可以反复执行。
- 地图目录**每次自动按时间新建**，不会和上一次重名，也不会被"已存在拒绝覆盖"卡住；保存时自动取"最新的那个"，不用手改名字。
- 三个角色：**小车**（SSH 终端）、**Windows 电脑**（PowerShell）、**Linux 电脑**（bash）。二选一用电脑：Windows 或 Linux。
- 跨机通信：`ROS_DOMAIN_ID` 两端必须一致 = **42**。

### 快捷键：小车输入 `1`，电脑输入 `2`
- 小车终端输入 `1` 回车，直接 `ros2 launch my_racecar distributed_mapping_vehicle.launch.py`（不检查是否已在运行，别重复敲）。
- 电脑 PowerShell 输入 `2` 回车，等同"A2-阈值"整块（`cd D:\ubuntu\slam` + `.\run_slam_windows.bat -Mode filtered`）。
- 电脑 PowerShell 输入 `3` 回车，保存地图（`cd D:\ubuntu\slam` + `.\run_slam_windows.bat -Save`），建图窗口保持运行，另开一个 PowerShell 敲。

**小车设置（只做一次，可重复执行不会重复写入）**：写进 `~/.bashrc`，之后每个新 SSH 窗口都能用。
```bash
sed -i '/# >>> racecar shortcut 1 >>>/,/# <<< racecar shortcut 1 <<</d' ~/.bashrc
cat >> ~/.bashrc <<'EOF'
# >>> racecar shortcut 1 >>>
racecar_mapping() (
  cd /home/racecar
  source /opt/ros/humble/setup.bash
  source /home/racecar/install/local_setup.bash
  export ROS_DOMAIN_ID=42
  export ROS_LOCALHOST_ONLY=0
  ros2 launch my_racecar distributed_mapping_vehicle.launch.py
)
alias 1='racecar_mapping'
# <<< racecar shortcut 1 <<<
EOF
source ~/.bashrc
type 1
```
最后一行显示 `1 is aliased to 'racecar_mapping'` 就成功了。删除：只运行第一行 `sed`，然后开新窗口。

**电脑设置（已做好）**：PowerShell 里直接输入 `2` 会被当成数字，所以在 `C:\Users\lzc2\Documents\WindowsPowerShell\Microsoft.PowerShell_profile.ps1` 里加了回车键处理：整行只有 `2` 时，换成上面的建图命令再执行。新开的 PowerShell 窗口才生效（已开的窗口执行 `. $PROFILE`）。只对 PowerShell 有效，cmd 里不行。删除：删掉这个文件里的那段。

---

### 第 0 步（小车，只在出现"found more than once"报错时做一次）
`ros2 launch` 报 `file ... was found more than once` = `install` 里有指向已搬走源文件的**悬空符号链接**。

```bash
cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
echo "=== 悬空链接（有输出就是要删的）==="
find /home/racecar/install/my_racecar/share/my_racecar -xtype l
echo "=== 删除它们（rm 只删链接本身，不碰源码）==="
find /home/racecar/install/my_racecar/share/my_racecar -xtype l -delete
echo "=== 确认新位置的源文件在 ==="
ls -l /home/racecar/src/my_racecar/mapping/launch/distributed_mapping_vehicle.launch.py
```
不需要重新编译。

---

### 第 1 步（小车）：启动底盘驱动 —— SSH 窗口一，保持运行
用安全版 `Run_car_safe.sh`（取代 `Run_car.sh`）：启动前清掉上次残留的驱动进程，`Ctrl+C` 时连根清理，避免出现重复的 `IMU_publisher`。**只按一次 Ctrl+C，多等几秒。**
首次使用要先上传，见 `上传到小车\README.md`（原来的 `bash Run_car.sh` 仍可用，没动）。
```bash
cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
bash /home/racecar/Run_car_safe.sh
```

### 第 2 步（小车）：启动里程计 —— SSH 窗口二，保持运行
只启动 `imu_encoder_mix`（发 `/encoder_imu_odom`）。**已不再启动 `odom_tf_adapter`**：电脑端 `scan_tf_bridge` 自己生成 TF，不用车端 `odom→base_footprint`，省约 14% CPU。已经在运行就不重复启动。
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

### 第 3 步（小车，可选）：自检
```bash
cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
timeout 5 ros2 topic hz /scan
timeout 5 ros2 topic hz /encoder_imu_odom
ros2 node list | grep -E "lslidar|imu_encoder_mix"
```
`/scan` 应稳定约 10 Hz。**车上不要同时跑 `mapping_only` / `race_mapping` / 其他 SLAM。**

---

### 电脑端 · 方案 A：Windows 电脑（PowerShell）

脚本会自动找出"和小车同网段（192.168.5.x）的网卡"并只让 DDS 走它（电脑有 WLAN / 代理隧道 / VMware 多块网卡时，不限定网卡会发现不了小车）。**必须先把电脑连到小车的 WiFi。**
没连小车 WiFi 时脚本会直接停下并列出当前网卡，提示"电脑没有和小车同网段的网卡"，不会再报一个含糊的"DDS 没通"。

### A1. 联机检查（约 8 秒出结果，不会卡住）
```powershell
cd D:\ubuntu\slam
.\run_slam_windows.bat -Check
```
通过标准：`发现的节点` 列出小车节点；`/scan`、`/encoder_imu_odom` 都 `收到：是`；`/map 发布者: 0`。
不通 → 看文末"排错表"。提示脚本被禁止运行时就用上面的 `.bat`（已带绕过策略参数）。

### 两种建图模式：原生（无过滤阈值） vs 带阈值（先看这个再选）
| | 原生 `native` | 带阈值 `filtered`（默认） |
|---|---|---|
| 参数 | **不设置** `occupancy_threshold` / `min_pass_through`，用 slam_toolbox 自带默认值（0.1 / 2） | `occupancy_threshold 0.20`、`min_pass_through 3` |
| 参数文件 | `mapping/config/mapper_native.yaml` | `mapping/config/mapper.yaml` |
| 细锥桶 | **保留最全**（模拟 1.5 m/s：7/12） | 较全（6/12，原来 0.35/4 只有 2/12） |
| 走动的人 | **会留下痕迹**（模拟里残留 3 格） | 基本不留痕迹（0 格） |
| 适合 | 场地里没人走动，只想把锥桶都建出来 | 有人走动、想要干净的图 |

两个文件**只有这两个参数不同**，其余一字不差，所以可以直接对比两张图。地图目录会自动带上模式后缀（`..._native` / `..._filtered`）。**两种模式都要慢速建图**（约 0.5 m/s），车速越快锥桶越容易丢。
以上数字来自模拟，实车效果请自己各建一张图对比。

### A2-原生. 开始建图：原生模式（不加过滤阈值）
```powershell
cd D:\ubuntu\slam
.\run_slam_windows.bat -Mode native
```

### A2-阈值. 开始建图：带阈值模式（默认，等同不写 -Mode）
```powershell
cd D:\ubuntu\slam
.\run_slam_windows.bat -Mode filtered
```
保存、监控、自动保存都和下面 A2/A3 完全一样，`-Mode` 可以和 `-NoOdom`、`-AutoSave` 等参数一起用。

### A2. 开始建图（SLAM + rviz2，窗口保持运行）
```powershell
cd D:\ubuntu\slam
.\run_slam_windows.bat
```
地图目录自动为 `D:\ubuntu\slam\maps\run_<日期_时分>`。推车**慢慢走**（转弯更慢）：**车速越高细锥桶越容易丢**（模拟实测 0.5 m/s 找回 6/6，1.5 m/s 只找回 2/12，见 `上传到小车\README.md`）。rviz 里应看到点云和逐渐长出的地图。

窗口里每 15 秒会打印一行 `[监控] 扫描…帧 | 里程计…条 | 地图已知…格 | 障碍…格`，出现下面的告警要处理：
- 红色“一条里程计都没收到”：小车 `imu_encoder_mix` 没在发 `/encoder_imu_odom`，此时位姿恒为原点，**地图几乎不会增长**。先启动里程计，或用 `-NoOdom` 重来。
- 黄色“地图 30 秒没增长”：车没动，或 SLAM 没跟上（里程计缺失/推太快）。
- **自动保存**：每 60 秒自动存一份到 `<地图目录>\autosave\`（`-AutoSave 30` 改间隔，`-AutoSave 0` 关）。只是保险，正式保存仍要 `-Save`。
- 没有里程计、纯激光匹配：`.\run_slam_windows.bat -NoOdom`
- 不开 rviz：`.\run_slam_windows.bat -NoRviz`

### A3. 保存地图（建图窗口保持运行，另开一个 PowerShell）
```powershell
cd D:\ubuntu\slam
.\run_slam_windows.bat -Save
```
自动保存到 `maps` 下**最新**的目录，成功会提示"第 1 趟建图已完整保存"，并生成 `run1\` 下的 `map.pgm map.png map.yaml map.raw map.posegraph map.data complete.json run.jsonl`。**直接看图双击 `map.png`**（Windows 打不开 `.pgm`；`.raw` 是原始栅格数据，不是图片）。`map.pgm + map.yaml` 是给 ROS 用的标准地图。
**不能只看 rviz 里有图就关窗口，必须看到上面这行提示。** 然后才可以在建图窗口按 `Ctrl+C`。
万一忘了保存：建图窗口结束时会红字提示“本趟没有正式保存”，并告诉你 `autosave\map.yaml` 的位置，用 `.\view_csv.bat -Map "<地图目录>\autosave\map.yaml"` 查看。

第 2、3 趟（比赛三趟建图）：`-RunIndex 2` / `-RunIndex 3`，并**用同一个地图目录**：
```powershell
cd D:\ubuntu\slam
$d = (Get-ChildItem D:\ubuntu\slam\maps -Directory | Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName
.\run_slam_windows.bat -RunIndex 2 -MapDir $d
```
保存同理：`.\run_slam_windows.bat -Save -RunIndex 2 -MapDir $d`。

### A4. 保存后用 rviz2 检查地图并叠加 CSV 路径（离线，不需要小车）
```powershell
cd D:\ubuntu\slam
$m = (Get-ChildItem D:\ubuntu\slam\maps -Recurse -Filter map.yaml | Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName
.\view_csv.bat -Map $m
```
障碍小、锥桶看不清：加 `-Inflate 1`（只影响显示，把障碍格加粗 1 格，地图文件不变）。再叠调试 CSV：`.\view_csv.bat -Map $m -Extra "D:\debug\a.csv","D:\debug\b.csv"`。

---

### 电脑端 · 方案 B：Linux 电脑（Ubuntu 22.04，已实车跑通）

### B1. 编译（只在代码更新后需要）
```bash
cd "/data/code/智能车/智能车(1)/my_racecar（复件）(1)"
source /opt/ros/humble/setup.bash
colcon build --packages-select my_racecar
```

### B2. 联机检查
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

### B3. 开始建图（SLAM + rviz2，窗口保持运行；目录自动按时间新建）
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
里程计不可靠时在最后一行末尾加 ` use_odom:=false`（推车要慢）。

### B3-原生. Linux：原生模式（不加过滤阈值）
```bash
cd "/data/code/智能车/智能车(1)/my_racecar（复件）(1)"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
MAP_DIR=~/maps/distributed_$(date +%Y%m%d_%H%M%S)_native
CFG=$(ros2 pkg prefix my_racecar)/share/my_racecar/mapping/config/mapper_native.yaml
echo "地图目录: $MAP_DIR"; echo "参数文件: $CFG"; ls -l $CFG
ros2 launch my_racecar distributed_mapping_pc.launch.py map_dir:=$MAP_DIR mapper_config:=$CFG
```

### B3-阈值. Linux：带阈值模式（默认 `mapper.yaml`）
```bash
cd "/data/code/智能车/智能车(1)/my_racecar（复件）(1)"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
MAP_DIR=~/maps/distributed_$(date +%Y%m%d_%H%M%S)_filtered
CFG=$(ros2 pkg prefix my_racecar)/share/my_racecar/mapping/config/mapper.yaml
echo "地图目录: $MAP_DIR"; echo "参数文件: $CFG"; ls -l $CFG
ros2 launch my_racecar distributed_mapping_pc.launch.py map_dir:=$MAP_DIR mapper_config:=$CFG
```
Linux 电脑要先把项目同步过去并重新 `colcon build --packages-select my_racecar`（安装目录会把 `mapping/config/` 下两个 yaml 都装进去）。保存用上面的 B4，不用区分模式。

### B4. 保存地图（建图窗口保持运行，另开终端；自动取最新目录）
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
成功会提示"第 1 趟建图已完整保存"。第 2、3 趟把 `--run-index` 和启动时的 `run_index:=` 都改成 2 / 3，`MAP_DIR` 用同一个目录。

---

### 排错表（按顺序查）
| 现象 | 原因 / 处理 |
|---|---|
| `ros2 launch` 报 `found more than once` | 做"第 0 步" |
| Windows `.\run_slam_windows.ps1` 提示找不到命令 | 前面要加 `.\`，或用 `.bat` |
| 提示"禁止运行脚本" | 用 `.bat` 版（带 `-ExecutionPolicy Bypass`） |
| 脚本提示"没连小车 WiFi"（红字，列出当前网卡） | 电脑连的是别的 WiFi（例如 10.x.x.x 网段）。切到小车 WiFi（192.168.5.x）再运行。此时默认路由会被代理隧道 Meta 接走，所以不要用路由判断 |
| `-Check` 显示"发现的节点: 无"（已连小车 WiFi） | 两端域号不同（必须 42）/ 加了 `-AllNics` / 小车 `Run_car.sh` 没启动 / 小车端第 1、2 步没跑 |
| rviz 没点云、Map 宽高 0，但节点都没报错 | 电脑没收到 `/scan`（DDS 走错网卡）。先 `-Check`；Windows 脚本默认已限定网卡 |
| `/map 发布者` 不为 0 | 车上或别处已有 SLAM / map_server 在发 `/map`，会冲突，先关掉 |
| 刷 `Message Filter dropping message` / `LaserRangeScan contains ...` | 不应出现（`scan_tf_bridge` 已处理）；出现说明没用桥接入口，检查用的是上面的启动命令 |
| 地图歪、有重影 | 推车太快；里程计与激光不一致时加 `-NoOdom` / `use_odom:=false` 慢速重建 |
| 保存提示"已存在，拒绝覆盖" | 用了旧目录。重新跑建图步骤（会新建目录），或换 `-RunIndex` |
| 监控红字"一条里程计都没收到" / 地图只有一小块 | 小车没发 `/encoder_imu_odom`（没启动 `imu_encoder_mix`）。位姿恒为原点时 SLAM 几乎不更新。先启动里程计，或 `-NoOdom` |
| `-Check` 提示"有重名节点" | 同一个驱动启动了多次（常见：IMU 的 `talker` 残留）。到小车上清残留，之后用 `Run_car_safe.sh` 启动 |
| 地图里锥桶很小或丢失 | 车速太快 + 过滤阈值过严。`mapper.yaml` 已调为 `occupancy_threshold 0.20 / min_pass_through 3`；建图那一圈开慢（约 0.5 m/s）；看图时加 `-Inflate 1` |
| 忘了保存就关了窗口 | 看 `<地图目录>\autosave\`（每 60 秒自动存的） |

### 状态说明
- Linux 方案：已实车跑通。
- Windows 方案：模拟小车端到端出图+保存通过；实车已能收到数据并建出图（2026-10-05 两趟：`run_20261005_1422`、`run_20261005_1458`）。
- 2026-10-05 的改进（自动保存、里程计告警、地图停滞告警、重复节点检测、锥桶阈值、显示加粗、`Run_car_safe.sh`）：**在模拟环境验证过，实车效果待验证**。
- 相关文件在 `windows建图linux建图\` 文件夹（源码备份与原理说明）。
