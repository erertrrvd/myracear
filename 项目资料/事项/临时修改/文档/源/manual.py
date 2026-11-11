# -*- coding: utf-8 -*-
"""《小车操作手册》内容：只放成品最终运行的命令。
调试、验证、排错命令放《改进与问题记录》（issues.py），这里只写"见《改进与问题记录》X"。
新增模块：在 CHAPTERS 里"附录"之前加一章，章末用 ('pkg', ...) 写清楚配套文件。
"""
UPDATED = "2026-11-04"

PROJ = "C:\\Users\\lzc2\\Desktop\\my_racecar（复件）"
HEAD = r"""cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0"""

# ------------------------------------------------------------------ 第 1 章
ENV = dict(title="第 1 章  环境与约定", intro="两台电脑、一辆车、三种终端；每条命令都标明在哪里运行。", blocks=[
    ("h2", "1.1 机器和网络"),
    ("table", ["项目", "内容"], [
        ["小车", "Atlas 200I DK A2（NPU 310B，CANN 7.0.RC1），Ubuntu 22.04，ROS2 Humble；root@192.168.5.100，ROS_DOMAIN_ID=42"],
        ["小车仓库", "/home/racecar/src/my_racecar（非裸仓库），工作空间 /home/racecar；摄像头 /dev/video0"],
        ["小车上的 Python", "root 的 python3 = conda 3.9（有 ais_bench，跑识别）；/usr/bin/python3 = 3.10（有 rclpy，跑 ROS）。两者不能放进同一个进程"],
        ["Windows 电脑", "项目 " + PROJ + "，remote car 指向小车仓库；建图环境 D:\\ubuntu\\slam（pixi）；视觉环境 D:\\light_ai\\venv"],
        ["Linux 电脑", "Ubuntu 22.04，工作空间 /data/code/智能车/智能车(1)/my_racecar（复件）(1)；用于建图和转 .om"],
        ["网络", "电脑连小车 WiFi，IP 在 192.168.5.x；不要开 VPN"],
    ], [1.6, 6.5]),
    ("h2", "1.2 三种终端"),
    ("table", ["终端", "用途", "注意"], [
        ["小车（SSH / MobaXterm，bash）", "启动驱动、里程计、视觉", "提示符前是 (base) 时 python3 是 conda 的；ROS 脚本用 /usr/bin/python3"],
        ["电脑 PowerShell", "Windows 建图、看图、git、scp", "没有 xargs；.ps1 被禁止运行时用对应的 .bat"],
        ["电脑 Git Bash", "哈希对比等需要 xargs 的命令", "路径写 /c/Users/..."],
    ], [2.4, 2.8, 4.5]),
    ("note", "命令块都自带 cd 和 source，整块复制就能跑，可以重复执行。不要把终端输出当命令粘回去。"),
    ("h2", "1.3 快捷键：小车 1，电脑 2 / 3 / 4 / a / b / c / d"),
    ("note", "2 和 4 是室外建图对照组（11-10 起）：两者都用 mapper_fast.yaml、投票、地面过滤和去畸变。2 的位姿完全用轮速 + IMU 的 /key1_odom，关闭激光扫描匹配和回环修正，激光只画图；4 保持原来的激光 SLAM 纠偏和 8 m 回环搜索。两个建图窗口不要同时开，应分别跑、分别保存后对比。完整建图步骤（小车 1 → 电脑 2 或 4 → 保存 3 或 a~d）见 3.2。建图时实时描黑（-LiveSolid）不再有快捷键，要用就手敲，见 3.4.1。"),
    ("table", ["在哪敲", "键", "等同于"], [
        ["小车终端", "1", "ros2 launch my_racecar distributed_mapping_vehicle.launch.py：启动建图专用的里程计（节点 key1_imu_encoder_mix，话题 /key1_odom，只给建图用；跑路径 / 录路径照旧自己起 imu_encoder_mix 发 /encoder_imu_odom，两个不冲突。不检查是否已在运行，别重复敲）"],
        ["电脑 PowerShell", "2", "cd D:\\ubuntu\\slam；.\\run_slam_windows.bat -Mode filtered -Fast -OdomOnly（轮速 + IMU 位姿对照建图，激光只画图；保存后自动描黑，见 3.4.1）"],
        ["电脑 PowerShell", "3", "cd D:\\ubuntu\\slam；.\\run_slam_windows.bat -Save（保存地图并描黑，另开一个窗口敲；2、4 都用它）"],
        ["电脑 PowerShell", "4", "cd D:\\ubuntu\\slam；.\\run_slam_windows.bat -Mode filtered -Fast（原来的高速激光 SLAM：回环搜索 8 m，见 3.6；保存后同样自动描黑）"],
        ["电脑 PowerShell", "a / b / c / d", "cd D:\\ubuntu\\slam；.\\run_slam_windows.bat -Save -Track A（B / C / D）：保存地图，并按赛道 A / B / C / D 拟合校正（抽中哪种敲哪个，小写；见 3.4.4）。3 是自动选最像的"],
    ], [1.6, 0.6, 7]),
    ("p", "小车设置（只做一次，重复执行不会重复写入；写进 /root/.bashrc，新 SSH 窗口都能用）："),
    ("code", r"""# 小车
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
type 1"""),
    ("p", "最后一行显示 1 is aliased to 'racecar_mapping' 即成功（2026-10-17 已在小车执行）。删除：只运行第一行 sed，再开新窗口。"),
    ("p", "电脑设置（已做好）：PowerShell 里 2 是数字，不能做别名，所以在 PowerShell 配置文件里加了回车键处理：整行只有 2、3 或 4 时换成对应命令再执行。新开的 PowerShell 窗口生效（VS Code 里的终端也一样），已开的窗口先运行 . $PROFILE。cmd 里无效。敲了只回显数字时见《改进与问题记录》二“VS Code 终端里敲 4 只回显数字 4”。"),
    ("pkg", [
        ["小车", "/root/.bashrc 末尾 racecar shortcut 1 段", "快捷键 1"],
        ["电脑", "C:\\Users\\lzc2\\Documents\\WindowsPowerShell\\Microsoft.PowerShell_profile.ps1", "快捷键 2、3、4（PSReadLine 回车键处理；11-10 起 2 = -Fast -OdomOnly，4 = -Fast）"],
        ["项目", "CLAUDE.md", "项目约定"],
    ]),
])

# ------------------------------------------------------------------ 第 2 章
GIT = dict(title="第 2 章  git：电脑和小车同步", intro="小车是非裸仓库，电脑先推到小车的 from-pc 分支，再在小车上快进合并。", blocks=[
    ("h2", "2.1 原则"),
    ("ul", [
        "不用 --force；不往小车当前分支 main 直接推。",
        "小车上有未提交改动时，先提交或用补丁备份；不用 checkout --、stash、reset --hard 去“清理”。",
        "合并只更新源码，不会自动编译；要生效需要自己 colcon build。",
        "src/pure_track_test.cpp 是 GBK 编码，不要在编辑器里重新保存；传输用补丁或 git。",
    ]),
    ("h2", "2.2 标准同步（两边都有新提交时）"),
    ("code", r"""# 1. 小车：先提交本地改动
cd /home/racecar/src/my_racecar && git add 要提交的文件 && git commit -m "说明" && git log --oneline -2"""),
    ("code", r"""# 2. 电脑 PowerShell：取回小车的提交并合并
cd "C:\Users\lzc2\Desktop\my_racecar（复件）"
git fetch car
git merge car/main --no-edit"""),
    ("code", r"""# 3. 电脑 PowerShell：提交自己的改动，推到小车的 from-pc 分支（要输入小车密码）
git add 文件
git commit -m "说明"
git push car main:from-pc"""),
    ("code", r"""# 4. 小车：快进合并（有分叉或冲突会直接拒绝，不改任何东西）
cd /home/racecar/src/my_racecar && git status --short && git merge --ff-only from-pc && git log --oneline -4"""),
    ("p", "合并后核对两边是否一致：见《改进与问题记录》一、git 同步 的调试命令。报错的处理也在那里。"),
    ("h2", "2.3 把小车上没提交的改动取回电脑（补丁）"),
    ("code", r"""# 小车：生成补丁
cd /home/racecar/src/my_racecar && git --no-pager diff -- 文件1 文件2 > /tmp/car_local_changes.patch && ls -l /tmp/car_local_changes.patch"""),
    ("code", r"""# 电脑 PowerShell（注意在电脑上运行）
scp root@192.168.5.100:/tmp/car_local_changes.patch "$env:USERPROFILE\Desktop\car_local_changes.patch"
cd "C:\Users\lzc2\Desktop\my_racecar（复件）"
git apply --check "$env:USERPROFILE\Desktop\car_local_changes.patch"
git apply "$env:USERPROFILE\Desktop\car_local_changes.patch"
"""),
    ("h2", "2.4 合并前打标签（便于回退）"),
    ("code", r"""# 小车
cd /home/racecar/src/my_racecar && git tag before-sync-$(date +%Y%m%d) && git tag | tail -3"""),
    ("warn", "确需回退时 git reset --hard 会丢掉已跟踪文件的未提交修改。先备份，再决定；步骤见《改进与问题记录》一、git 同步。"),
    ("pkg", [
        ["电脑", "git remote car = ssh://root@192.168.5.100/home/racecar/src/my_racecar", "电脑仓库指向小车"],
        ["小车项目", "分支 from-pc", "电脑推过来的分支，合并后可保留"],
    ]),
])

# ------------------------------------------------------------------ 第 3 章
MAP = dict(title="第 3 章  分布式建图", intro="小车发雷达 /scan 和建图专用里程计 /key1_odom（快捷键 1 起的那一个），电脑用 slam_toolbox 建图，地图存电脑。Windows 和 Linux 两套电脑端逻辑相同，二选一。", blocks=[
    ("h2", "3.1 分工"),
    ("code", r"""小车                                      电脑
Run_car_safe.sh：雷达 /scan、IMU /IMU_data、  scan_tf_bridge：/scan -> 重采样 720 点 -> 逐点去畸变
  编码器 /encoder                                 -> 删地面线 -> /scan_pc；按扫描时间戳发 odom_pc->base_pc
快捷键 1：imu_encoder_mix -> /key1_odom  --DDS/WiFi-->  slam_toolbox（async）-> /map；rviz2；save_mapping
（车上唯一的里程计；跑路径、录路径也用它）       -Odom ekf 时：ekf_input + ekf_node 融合 /encoder + /IMU_data
固定路径、行驶控制仍在车上"""),
    ("p", "电脑不用车端 /tf，也不向车发任何运动指令。车上不要同时跑 mapping_only / race_mapping 等其他 SLAM。"),
    ("h2", "3.2 一次完整建图（按顺序，Windows 电脑）"),
    ("p", "建图对照有两种，只差电脑端第 3 步，其余步骤完全一样；不要同时运行："),
    ("table", ["", "快捷键 2：轮速 + IMU 位姿对照", "快捷键 4：原来的高速激光 SLAM"], [
        ["电脑命令", ".\\run_slam_windows.bat -Mode filtered -Fast -OdomOnly", ".\\run_slam_windows.bat -Mode filtered -Fast"],
        ["参数文件与处理", "mapper_fast.yaml；投票、地面过滤、去畸变与 4 相同", "同左"],
        ["位姿", "完全用轮速 + IMU 的 /key1_odom；激光只画图，不做扫描匹配和回环修正", "轮速 + IMU 提供预测，激光扫描匹配和回环继续纠偏"],
        ["地图目录", "maps\\run_<时间>_fast_odom_only", "maps\\run_<时间>_fast"],
        ["rviz 里看到", "里程计位姿画出的原图", "SLAM 纠偏后的原图"],
        ["保存（快捷键 3 或 a~d）得到", "run1\\map.* 原图，存完自动描黑 map_solid.*、赛道拟合 map_track.*、报告 report.html", "同左"],
        ["怎么看对照", "corr 为 0 是这个模式的预期，不代表没有里程计漂移；看原图两圈是否重合", "看原图、corr 突跳和两圈重合情况，与 2 分开跑后对比"],
    ], [2, 4, 4.6]),
    ("p", "第 1 步　小车 SSH 窗口一（保持运行）：底盘驱动。安全版脚本，启动前清残留；结束时 Ctrl+C 只按一次、多等几秒。首次使用先按第 4 章上传。"),
    ("code", HEAD + "\nbash /home/racecar/Run_car_safe.sh"),
    ("p", "第 2 步　等约 10 秒，小车 SSH 窗口二（保持运行）：里程计。敲 1 回车，等同下面整块（启动建图专用里程计 key1_imu_encoder_mix，发 /key1_odom；不检查是否已在运行，别敲两次）："),
    ("code", "# 小车 SSH 窗口二，等同快捷键 1\n" + HEAD + "\nros2 launch my_racecar distributed_mapping_vehicle.launch.py"),
    ("p", "第 3 步　电脑 PowerShell 窗口一（保持运行）：开始建图，两种二选一。电脑必须先连小车 WiFi；第一次或换了网络先做联机检查（《改进与问题记录》八、调试命令 中的 -Check）。"),
    ("code", r"""# 电脑 PowerShell 窗口一：轮速 + IMU 位姿对照。敲 2 回车，等同下面整块
cd D:\ubuntu\slam
.\run_slam_windows.bat -Mode filtered -Fast -OdomOnly"""),
    ("code", r"""# 电脑 PowerShell 窗口一：原来的高速激光 SLAM。敲 4 回车，等同下面整块
cd D:\ubuntu\slam
.\run_slam_windows.bat -Mode filtered -Fast"""),
    ("p", "第 4 步　推车或开车跑赛道（对照时让 2 和 4 分别跑同一路线、同一速度）。边跑边看两处，都不用另敲命令："
          "① rviz 窗口（第 3 步自动弹出）：灰底上白色空地从车周围向外扩，黑点是墙和锥桶，车走到哪图长到哪。"
          "② 电脑 PowerShell 窗口一（建图窗口）：每 15 秒一组青色 [监控]，第一行“地图已知 N 格”跑起来后应不断变大；第二行是地面线过滤，怎么看见 3.4.3。"),
    ("p", "让车自己跑路径来建图时，小车另开 SSH 窗口三照平时启动跑路径（跑路径代码 11-04 已恢复原样：自己起 imu_encoder_mix 发 /encoder_imu_odom，跑路径用它；建图只用第 2 步的 /key1_odom，两个话题分开，互不干扰）。ab 照平时（a / b / none）。跑完在这个窗口按 Ctrl+C 停车，再做第 5 步保存："),
    ("code", "# 小车 SSH 窗口三：自动跑路径（和平时一样）\n" + HEAD + "\nros2 launch my_racecar move_test.launch.py ab:=a"),
    ("p", "跑起来后看电脑建图窗口一：不能出现红字“车上有 N 个 /key1_odom 发布者”或“/key1_odom N 条/s（一个里程计约 50）”（只有快捷键 1 敲了两次才会出现）。出现了按《改进与问题记录》二“两个里程计同时在发”查。"),
    ("p", "第 5 步　电脑另开 PowerShell 窗口二：保存。2 和 4 都用它。知道抽中哪种赛道就敲 a / b / c / d（保存并按赛道 A / B / C / D 拟合，见 3.4.4），不知道就敲 3（自动选最像的）。敲 3 等同下面整块；就在这个窗口二里看到“第 1 趟建图已完整保存：<目录>”才算成功（没看到就别关窗口一）："),
    ("code", r"""# 电脑 PowerShell 窗口二（建图窗口保持运行），等同快捷键 3；快捷键 a 等同于最后一行换成 .\run_slam_windows.bat -Save -Track A
cd D:\ubuntu\slam
.\run_slam_windows.bat -Save"""),
    ("p", "正常：窗口二第一行是“保存到 <目录> (run1)，拟合：赛道 A（快捷键 a） ...”（敲 3 时是“赛道自动选最像的（快捷键 3）”）。"),
    ("p", "第 6 步　收尾：先在电脑建图窗口（窗口一）按 Ctrl+C，再停小车两个窗口：各按一次 Ctrl+C，看到提示符 (base) root@davinci-mini:~# 重新出现再关窗口。"),
    ("note", "两个建图窗口（2 和 4）不要同时开；快捷键只在新开的 PowerShell 窗口生效。启动后自检（/scan 约 10 Hz 等）见《改进与问题记录》八、调试命令。"),
    ("h2", "3.3 Windows 电脑：建图参数"),
    ("table", ["", "带阈值 filtered（默认，快捷键 2、4 都用它）", "原生 native"], [
        ["参数", "occupancy_threshold 0.20、min_pass_through 3", "不设置，用 slam_toolbox 默认 0.1 / 2"],
        ["参数文件", "mapping/config/mapper.yaml", "mapping/config/mapper_native.yaml"],
        ["细锥桶（模拟 1.5 m/s）", "6/12", "7/12，保留最全"],
        ["走动的人", "基本不留痕迹", "会留痕迹"],
        ["适合", "有人走动、要干净的图", "场地没人走动、只想把锥桶建全"],
    ], [1.8, 4, 4]),
    ("code", r"""# 电脑 PowerShell：原生模式（没有快捷键；保存后同样自动描黑）
cd D:\ubuntu\slam
.\run_slam_windows.bat -Mode native"""),
    ("ul", [
        "地图目录自动为 D:\\ubuntu\\slam\\maps\\run_<日期_时分>_<模式>，不会重名。",
        "地图目录自动加后缀：-Fast 是 _fast，-Odom ekf 再加 _ekf（如 run_20261105_1500_fast_ekf），方便对比。",
        "车速越快细锥桶越容易丢；比赛要高速（约 3 m/s）建图时用 3.6 的 -Fast。",
        "建图窗口每 15 秒打印 [监控]：第一行扫描 / 里程计 / 地图格数，第二行地面线。红字“一条里程计都没收到”、红字“两个里程计”、黄字“地图 30 秒没增长”的处理见《改进与问题记录》二、分布式建图。",
        "每 60 秒自动存一份到 <地图目录>\\autosave\\（只是保险）。",
        "其他参数：-NoOdom（不用里程计，纯激光）、-NoRviz、-LiveSolid（建图时实时描黑，见 3.4.1）、-Ground off / fixed（地面线过滤，见 3.4.3）、-NoVote（不画两种投票地图，见 3.4.5）、-Fast / -Odom ekf / -NoDeskew（高速建图，见 3.6）、-Track A|B|C|D（赛道拟合指定赛道，见 3.4.4）、-AutoSave 30（改间隔，0 关）、-RunIndex 2 -MapDir <目录>（第 2、3 趟）。",
    ]),
    ("h2", "3.4 Windows 电脑：保存地图"),
    ("p", "命令见 3.2 第 5 步（快捷键 3）。"),
    ("warn", "必须看到“第 1 趟建图已完整保存”再在建图窗口按 Ctrl+C。只看 rviz 里有图就关窗口，地图没存。"),
    ("p", "保存在 maps 下最新目录的 run1\\：map.png（双击看图）、map.pgm + map.yaml（ROS 用的标准地图）、map.raw（原始栅格数据，不是图片）、map.posegraph、map.data、complete.json、run.jsonl；"
          "另外还有存完自动描黑出来的 map_solid.png / .pgm / .yaml（锥桶画成实心圆的图）和 solid.json（锥桶数、锥桶位置）；"
          "11-05 起还有两种投票地图：map_vote_raw.png / .pgm / .yaml / .raw（只投票，没做地面线剔除）和 map_vote.*（投票 + 地面剔除），见 3.4.5。"),
    ("h2", "3.4.1 描黑：存图后自动做（快捷键 2、4 都是）；建图时实时做（-LiveSolid，手敲）"),
    ("p", "车快时 slam_toolbox 的锥桶只剩零散一两个点，看起来淡、模糊。描黑 = 在 SLAM 原图上把小而孤立的锥桶点团换成大小一致的实心圆，墙和其他障碍保持原样。只用来看，不影响 SLAM 定位和原图 map.*。"),
    ("p", "11-03 起默认在保存时做：按 3 保存，原图存完后同一个窗口二里接着打印一行，样子如下（不是要敲的命令）："),
    ("code", r"""第 1 趟建图已完整保存：D:\ubuntu\slam\maps\run_20261103_2002_filtered\run1
描黑完成：锥桶 41 个（半径 0.08 m），没加工的小团 354 个（靠墙 / 靠边 / 周围杂点多，保持原图）。看图：D:\ubuntu\slam\maps\run_20261103_2002_filtered\run1\map_solid.png"""),
    ("p", "正常：出现“描黑完成”这一行，双击它给出的 map_solid.png，赛道上的锥桶是一个个大小一样的黑圆点。不正常：出现红字“描黑后处理失败（原图已存好，不受影响）”——原图照样能用，把整行红字贴出来，按《改进与问题记录》二“存图后描黑失败”处理。"),
    ("ul", [
        "只加工赛道：离墙、离房间外未探明区域、离贴着外边的墙断块 / 门框 1 m 以内的点团不动，按原图；周围还有别的点的（杂物堆）也不动。",
        "存图后描黑没有扫描数据，锥桶半径固定 0.08 m；要改用下面的命令加 --radius 0.10 重做一遍。",
        "原图里一个点都没有的锥桶补不出来；走动的人由原图的 filtered 阈值过滤。",
        "-LiveSolid（11-04 起没有快捷键，电脑 PowerShell 手敲 cd D:\\ubuntu\\slam 再 .\\run_slam_windows.bat -Mode filtered -LiveSolid）建图时 rviz 里就有 Map Solid 一层（左侧 Displays 里 Map Solid 打勾），半径按近处单帧扫描拟合；保存时直接用实时那一份，不再重做。",
        "Linux：launch 默认 solid:=false，保存（ros2 run my_racecar save_mapping）后同样自动描黑；要实时描黑在 launch 最后加 solid:=true。",
    ]),
    ("p", "对已经存好的地图（或 autosave 目录）手动描黑 / 重做："),
    ("code", r"""# 电脑 PowerShell：给最新一次地图的 run1 重新描黑（--force 覆盖已有的 map_solid）
cd D:\ubuntu\slam
$d = (Get-ChildItem D:\ubuntu\slam\maps -Directory | Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName
pixi run --frozen python "C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\scripts\solidify_map" "$d\run1" --force"""),
    ("p", "正常：打印“描黑完成：锥桶 N 个……看图：<路径>\\map_solid.png”。只有 autosave 没有 run1 时，把 \"$d\\run1\" 换成 \"$d\\autosave\"。"),
    ("h2", "3.4.2 建图报告：每趟自动生成，一页看清建得怎么样"),
    ("p", "不用另敲命令：按 3 保存后，保存窗口打印“建图报告：<目录>\\run1\\report.html”并自动用浏览器打开；没保存就关建图窗口（Ctrl+C）时，给 autosave 生成一份并打开。"
          "报告里：结论表（每项写正常 / 不正常 / 参考）、后处理图（描黑后的地图 + 红色锥桶 + 蓝色车轨迹，绿点起点、黑点终点）、SLAM 修正量曲线、原图、每 5 秒明细。"),
    ("table", ["结论表这一项", "正常", "不正常说明什么"], [
        ["SLAM 突跳", "0 次（5 s 内修正都没超过 1 m / 10°）", "那几秒地图错位、锥桶重影；对着曲线里的红虚线看是什么时候"],
        ["里程计是不是只有一个", "车停、车动都约 50 条/s，车上发布者 1 个", "车动时 70 条/s 以上（两个时约 90），或发布者 2 个：有两个来源在发同一个话题，见《改进与问题记录》二“两个里程计同时在发”"],
        ["里程计晚到（WiFi）", "“对不上时刻、只能用旧位姿的扫描”很少（≤ 2%）；扣住等里程计几十上百帧、最长零点几秒是正常的", "旧位姿的帧多：WiFi 差、里程计晚到超过 1 s，转弯时 SLAM 来回摆、报突跳。车离路由器近一点 / 换 5 GHz，见《改进与问题记录》二“建图突跳：里程计经 WiFi 晚到”"],
        ["数据有没有断", "0 段","中途有整整 5 s 一条都没收到、之后又恢复：网络断过（结尾关了快捷键 1 之后没数据不算）"],
        ["地面线", "操场上认出（给出法向、距离），法向接近 0°", "操场有跟车走的直线却没认出：下次按报告给的数 -Ground fixed；法向在侧面 / 身后：认成了挡板，见 3.4.3"],
        ["描黑", "锥桶 N 个", "没有：描黑失败，见 3.4.1"],
    ], [3, 4, 5.5]),
    ("p", "以前的地图、或者想重新生成：给最新一次地图生成报告并打开（-MapDir 指定别的目录）："),
    ("code", r"""# 电脑 PowerShell
cd D:\ubuntu\slam
.\run_slam_windows.bat -Report"""),
    ("h2", "3.4.3 地面线过滤（默认开，不用敲命令）"),
    ("p", "雷达略往下斜时，扫描面在车前约 3 m 打到地面，地图里会多一条跟着车走的直线：静止时是假墙，车动时 SLAM 拿它匹配，以为车没动，地图被带偏。"
          "建图窗口（敲 2 / 4 的那个 PowerShell）里的扫描桥会自己认出这条线并删掉：雷达坐标系里一直不动的长直线，而且车后来开过了它之前所在的位置（地面线跟着车走，车开得过去；比赛挡板车永远开不过去），才是地面线。"
          "11-05 改的判据：旧版只看“车朝它累计走 1 m 或累计转 0.5 rad”，沿弯道挡板转弯、贴着挡板晃动都会累计够，把挡板当成地面线、连线外整片一起删掉（11-03、11-05 实测认出的法向 -28°~-47°、129°，都是侧面 / 身后的挡板）。"
          "认出前什么都不删；室内雷达水平、没扫到地面时一直认不出，是正常的。"),
    ("p", "在哪看：建图窗口每 15 秒的 [监控] 第三行，样子如下（不是要敲的命令）："),
    ("code", r"""[监控] 地面线：还没认出（车要动起来才学得到；雷达水平、没扫到地面时一直没有是正常的）
[监控] 地面线：法向 1.5°、距雷达 3.05 m，5s 内删掉 4200 个点"""),
    ("table", ["看哪一项", "正常", "不正常 / 怎么办"], [
        ["操场开出约 1 m 后", "变成“法向 X°、距雷达 Y m”，Y 约 2~5 m，X 接近 0°（正前方）", "开了几十米还是“还没认出”，而 rviz 车前仍有跟车走的直线：见《改进与问题记录》二“地面假墙”卡片"],
        ["rviz 车前", "跟车走的那条直线（红色扫描点）消失；锥桶的红点还在", "锥桶也大片消失：Y 比实际锥桶近，先 -Ground off 重建，把 [监控] 那行贴出来"],
        ["法向 X", "接近 0°（车正前方），因为雷达是往前低头", "X 在 ±30° 以外（侧面、身后）：认成了挡板 / 墙。保存后看报告“两种投票地图”：右图（投票 + 地面剔除）比左图（只投票）少一整段挡板就是认错了，下次 -Ground off，把 [监控] 那行贴出来"],
        ["室内", "一直“还没认出”", "室内认出了地面线、墙缺了一块：-Ground off"],
    ], [2.6, 4.2, 5.5]),
    ("p", "静止建图（车不动学不出来）或想固定用某个值：把 [监控] 里认出的两个数填进去："),
    ("code", r"""# 电脑 PowerShell 窗口一：直接给定地面线（数值换成自己 [监控] 里看到的）
cd D:\ubuntu\slam
.\run_slam_windows.bat -Mode filtered -Ground fixed -GroundRho 3.05 -GroundDeg 1.5"""),
    ("code", r"""# 电脑 PowerShell 窗口一：关掉地面线过滤
cd D:\ubuntu\slam
.\run_slam_windows.bat -Mode filtered -Ground off"""),
    ("p", "Linux：launch 最后加 ground_filter:=off，或 ground_filter:=fixed ground_rho:=3.05 ground_theta_deg:=1.5。"),
    ("h2", "3.4.5 两种投票地图：只投票 / 投票 + 地面剔除（默认开，rviz 实时看，保存时一起存）"),
    ("p", "“投票”= slam_toolbox 判障碍的办法：每格数被命中几次、被射线经过几次，经过 3 次以上才判断，命中比例 ≥ 20% 是障碍（偶尔扫到一次的人、颠簸时的地面回波票数不够，被剔掉）。"
          "“地面剔除”= 3.4.3 的地面线过滤。建图时 vote_map 节点同时画两张图，只用来看和对比，不影响 SLAM："),
    ("table", ["图", "用的扫描", "看什么"], [
        ["Map Vote Raw（只投票）→ 保存为 map_vote_raw.*", "没做地面线过滤的扫描", "挡板、墙、锥桶都在（有地面线时也会多一条线）"],
        ["Map Vote（投票 + 地面剔除）→ 保存为 map_vote.*", "SLAM 实际用的扫描", "和左图比，只应少一条跟车走的地面线"],
    ], [4.2, 3, 5]),
    ("ul", [
        "rviz 里看：左侧 Displays 里 Map Vote Raw (vote only)、Map Vote (vote + ground removed) 默认不打勾，打勾就显示（建议同时把 Map 的勾去掉，一次只看一层）。约每 1 秒刷新一次。",
        "位姿是实时优化的：vote_map 跟着 slam_toolbox 位姿图（回环、匹配纠偏后每个节点的新位置）把历史扫描撤票、按新位姿重投，回环后两层也会跟着拉正，不会留下旧位置的斜线。",
        "保存（3 / a~d）后报告 report.html 里有“两种投票地图”一节，两张图左右并排，下面写障碍格各多少、地面剔除删掉多少。右图少一整段挡板 = 地面线认错了，见 3.4.3 表格最后一行。",
        "不想要（省电脑 CPU）：建图命令加 -NoVote；Linux launch 加 vote:=false。没开时保存照常，只是少这两张图。",
    ]),
    ("h2", "3.4.4 赛道拟合：和规则里的 4 种赛道对齐并强制校正（自动做）"),
    ("p", "比赛赛道是规则里 A、B、C、D 四种随机抽一种（16.8 m × 5.5 m，两头半圆，车道宽 1.5 m）。按 3 保存、描黑之后自动做，一律强制校正（和赛道有差异的地方、建图偏移都拉回去）："),
    ("ul", [
        "拿描黑认出的锥桶和四种赛道分别对齐（平移、旋转、比例），选最像的那种；知道抽中哪种时用 -Track 指定（见下）。",
        "把整张地图拉成那种赛道的形状：先拉直整体弯曲（两圈不重合、漂移），再贴细节。",
        "锥桶都拉到赛道线上重画成 8 cm 实心圆；135° 折线区（规则图黄线）有锥桶就拉到黄线上，没有也行；挤在一起的（重影、两圈各一份）合成一个。",
        "赛道范围内只留这些锥桶，别的黑点（重影、碎点）清掉；场外的人、树、杂物不动。",
        "原图 map.* 和 map_solid.* 不动，校正结果另存 map_track.*。",
    ]),
    ("p", "在哪看：保存窗口二里描黑那行后面接着一行，样子如下（不是要敲的命令）；建图报告结论表里也有“赛道拟合”一行，下面有一张叠加图（蓝线外圈、红线内圈、黄线折线区、绿点是校正后的锥桶）："),
    ("code", r"""赛道拟合：最像赛道 A（分数 0.546，其次 B 0.547）；比例 1.006；和规则差异大（分数 > 0.45），已强制校正；49/71 个锥桶拉到赛道线上（拉之前离线中位 0.16 m、最远 0.76 m），挤在一起的合并后 45 个；赛道内贴不上线的重影擦掉 4 个，场外 18 个不动；和 B 分不清，不对就加 --track 重做。校正后的地图：...\run1\map_track.png，叠加图：...\run1\track_fit.png"""),
    ("table", ["看哪一项", "正常", "不正常 / 怎么办"], [
        ["分数", "0.3 以下（假数据认对时 0.16~0.29）", "超过 0.45：写“和规则差异大，已强制校正”，形状是硬拉过去的。练习场地摆法和规则图不同时就是这样（11-03 操场 0.5~0.75，六趟分别认成了 A/B/C/D）：用 -Track 固定一种"],
        ["track_fit.png", "绿点都在蓝 / 红 / 黄线上，数量和实际锥桶差不多", "绿点明显少：描黑认出的锥桶少，看描黑那一行；形状不对：认错了赛道，用 -Track 指定"],
        ["“和 X 分不清”", "不出现，或已用 -Track 指定", "A/B、C/D 只差左头内圈是直角还是圆弧：按实际抽中的用 -Track 指定"],
    ], [2.4, 4.2, 5.8]),
    ("p", "输出（都在 run1\\）：map_track.png / .pgm / .yaml（校正后的地图，坐标原点在赛道外框左下角外 1 m）、track_fit.png（叠加图）、track_fit.json（选的赛道、四种的分数、锥桶个数）。"),
    ("p", "保存时指定赛道：电脑 PowerShell 窗口二直接敲 a / b / c / d 回车（抽中 A 敲 a，依此类推；代替快捷键 3），等同下面整块（以 C 为例）："),
    ("code", r"""# 电脑 PowerShell 窗口二：保存并按赛道 C 拟合，等同快捷键 c
cd D:\ubuntu\slam
.\run_slam_windows.bat -Save -Track C"""),
    ("code", r"""# 电脑 PowerShell：已经存好的最新一次地图，按指定赛道重做拟合和报告
cd D:\ubuntu\slam
.\run_slam_windows.bat -Report -Track C"""),
    ("p", "正常：保存窗口里那行开头是“赛道拟合：指定赛道 C”；-Report 重新生成报告并用浏览器打开，结论表“赛道拟合”一行写“指定赛道 C”。"),
    ("note", "模板里两个弯（A/B 上边左半、C/D 下边右半）的形状是从规则截图量的锥桶位置，误差约 0.2 m；直线、半圆按标注尺寸。拿到规则原文的精确坐标后改 fit_track 开头的 DIP_OUT / DIP_IN。"),
    ("p", "第 2、3 趟（比赛三趟建图）用同一个地图目录："),
    ("code", r"""# 电脑 PowerShell 窗口一：第 2 趟建图（对应 2；要实时描黑在最后加 -LiveSolid）
cd D:\ubuntu\slam
$d = (Get-ChildItem D:\ubuntu\slam\maps -Directory | Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName
.\run_slam_windows.bat -Mode filtered -RunIndex 2 -MapDir $d"""),
    ("code", r"""# 电脑 PowerShell 窗口二：保存第 2 趟（两种都用这个）
cd D:\ubuntu\slam
$d = (Get-ChildItem D:\ubuntu\slam\maps -Directory | Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName
.\run_slam_windows.bat -Save -RunIndex 2 -MapDir $d"""),
    ("p", "第 3 趟把上面两块里的 2 改成 3。"),
    ("p", "保存后叠加 CSV 路径查看地图：见第 5 章。"),
    ("h2", "3.5 Linux 电脑：建图和保存"),
    ("code", r"""# Linux 电脑：编译（代码更新后才需要）
cd "/data/code/智能车/智能车(1)/my_racecar（复件）(1)"
source /opt/ros/humble/setup.bash
colcon build --packages-select my_racecar"""),
    ("code", r"""# Linux 电脑：开始建图（对应 Windows 的 2；保存后自动描黑）
# 原生模式：把 mapper.yaml 换成 mapper_native.yaml，目录后缀 _filtered 改 _native
cd "/data/code/智能车/智能车(1)/my_racecar（复件）(1)"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
MAP_DIR=~/maps/distributed_$(date +%Y%m%d_%H%M%S)_filtered
CFG=$(ros2 pkg prefix my_racecar)/share/my_racecar/mapping/config/mapper.yaml
echo "地图目录: $MAP_DIR"; ls -l $CFG
ros2 launch my_racecar distributed_mapping_pc.launch.py map_dir:=$MAP_DIR mapper_config:=$CFG solid:=false"""),
    ("code", r"""# Linux 电脑：开始建图，建图时实时描黑（对应 Windows 的 4）
cd "/data/code/智能车/智能车(1)/my_racecar（复件）(1)"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
MAP_DIR=~/maps/distributed_$(date +%Y%m%d_%H%M%S)_filtered
CFG=$(ros2 pkg prefix my_racecar)/share/my_racecar/mapping/config/mapper.yaml
echo "地图目录: $MAP_DIR"; ls -l $CFG
ros2 launch my_racecar distributed_mapping_pc.launch.py map_dir:=$MAP_DIR mapper_config:=$CFG solid:=true"""),
    ("note", "Linux 要先把项目同步过去并重新 colcon build --packages-select my_racecar（11-03 新增 solidify_map，存图后描黑要用）。"),
    ("p", "里程计不可靠时在最后一行末尾加 use_odom:=false（推车要慢）。RViz：Fixed Frame 选 map，添加 /map、/scan_pc、TF。"),
    ("code", r"""# Linux 电脑：保存（另开终端，自动取最新目录）
cd "/data/code/智能车/智能车(1)/my_racecar（复件）(1)"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
MAP_DIR=$(ls -dt ~/maps/distributed_* | head -n 1)
echo "保存到: $MAP_DIR"
ros2 run my_racecar save_mapping --run-index 1 --map-dir $MAP_DIR
ls -l $MAP_DIR/run1"""),
    ("p", "保存成功：这个终端打印“第 1 趟建图已完整保存：<目录>”，接着打印“描黑完成：锥桶 N 个……”，最后的 ls 列出 map.pgm、map.yaml、map.png、map_solid.*、solid.json 等。"),
    ("p", "第 2、3 趟：启动加 run_index:=2，保存用 --run-index 2，MAP_DIR 用同一个目录。"),
    ("p", "Linux 电脑端和 Windows 一样默认开逐点去畸变（deskew:=true stamp_at_end:=true，见 3.6）；高速两圈参数把 CFG 里的 mapper.yaml 换成 mapper_fast.yaml。EKF 融合模式目前只有 Windows 入口有。"),
    ("h2", "3.6 高速建图（约 3 m/s、跑两圈）：去畸变、-Fast、-Odom ekf"),
    ("p", "比赛要在高速下建图。车跑 3 m/s 时，雷达转一圈（0.1 s）里车已走了 30 cm，两圈下来里程计也会漂。下面三样针对这个，前一样默认就开，后两样加参数："),
    ("table", ["是什么", "怎么开", "作用"], [
        ["逐点去畸变", "默认开（-NoDeskew 关）", "按里程计的车速、角速度把一圈里每个点挪回同一时刻。假数据 3 m/s + 转弯：点的误差中位数 7.9 cm → 0.3 cm"],
        ["高速两圈参数 mapper_fast.yaml", "-Fast", "回环搜索距离 4 m → 8 m：第二圈回到第一圈的地方时，更远的偏差也能拉回去。其余（含移动目标剔除 0.20/3）同默认。还没用实车包对比过"],
        ["EKF 融合（轮速 + IMU，雷达由 SLAM 匹配）", "-Odom ekf", "电脑上 robot_localization 融合 /encoder 车速和 /IMU_data 角速度，代替 /key1_odom 做位姿。还没上车测"],
    ], [3, 2.4, 6.5]),
    ("code", r"""# 电脑 PowerShell 窗口一：高速两圈建图，敲 4 回车等同下面整块（代替第 3 步的 2）
cd D:\ubuntu\slam
.\run_slam_windows.bat -Mode filtered -Fast"""),
    ("code", r"""# 电脑 PowerShell 窗口一：高速两圈 + EKF 融合
cd D:\ubuntu\slam
.\run_slam_windows.bat -Mode filtered -Fast -Odom ekf"""),
    ("p", "EKF 模式要看两处（都在建图窗口一，不用另敲命令）："),
    ("ul", [
        "每 5 秒一行“5s 内：轮速 N 条、IMU N 条、EKF 输出 N 条”：三个数都大于 0 才对。出现红字“没收到 /encoder、/IMU_data”：小车第 1 步的驱动没在发，或电脑收不到，见《改进与问题记录》二“EKF 模式没有输入”。",
        "车转弯后出现一行“航向自检：IMU 积分 X°，/key1_odom Y°，比例 K”：K 在 0.8~1.25 写“正常”。红字说“IMU 发的是 度/秒”或“方向反了”时，按它给的数加 -GyroScale 重开，例如 .\\run_slam_windows.bat -Mode filtered -Fast -Odom ekf -GyroScale -1。写“对不上”就先别用 EKF。",
        "EKF 模式下快捷键 1 仍要敲：建图桥照样盯着 /key1_odom 查有没有两个里程计。",
    ]),
    ("pkg", [
        ["项目", "mapping/launch/distributed_mapping_vehicle.launch.py", "小车端（快捷键 1）：只起 imu_encoder_mix，输出 remap 成 /key1_odom"],
        ["项目", "mapping/launch/distributed_mapping_pc_windows.launch.py", "Windows 电脑端入口（不需要编译 my_racecar）：deskew、odom_source（key1 / ekf）、imu_gyro_scale 参数"],
        ["项目", "mapping/launch/distributed_mapping_pc.launch.py", "Linux 电脑端入口"],
        ["项目", "mapping/scripts/scan_tf_bridge", "收 /scan → 重采样 720 点 + 逐点去畸变 + 删地面线 + 按扫描时间戳发 TF；盯 /key1_odom 的发布者个数和条数 / s"],
        ["项目", "mapping/scripts/ekf_input", "EKF 模式：用小车时间（发 /ekf_clock，ekf_node 跟着走）、填协方差、航向自检；/encoder、/IMU_data -> ekf_node -> /ekf_odom"],
        ["项目", "mapping/config/ekf.yaml", "EKF 参数（robot_localization）：轮速 vx、vy=0 + IMU 角速度 z"],
        ["项目", "mapping/config/mapper_fast.yaml", "高速两圈 SLAM 参数（-Fast）：回环搜索 8 m"],
        ["项目", "mapping/scripts/fit_track", "赛道拟合：描黑后的锥桶和规则 A/B/C/D 对齐、强制校正、锥桶重画到线上 → map_track.*、track_fit.png / .json（保存时自动做；-Track 指定）"],
        ["项目", "mapping/scripts/mapping_status_logger", "写 .run1-live.jsonl：扫描/里程计/地图格数，odom/slam/corr 位姿"],
        ["项目", "mapping/scripts/map_autosave", "每 60 秒自动存到 autosave\\"],
        ["项目", "mapping/scripts/save_mapping", "正式保存（含 map.png；存完自动描黑出 map_solid.* 和 solid.json）"],
        ["项目", "mapping/scripts/solidify_map", "存图后描黑：存好的地图 -> map_solid.*（锥桶实心圆）"],
        ["项目", "mapping/scripts/map_report", "建图报告：report.html（结论表、描黑图 + 轨迹、corr 曲线），保存 / 关建图窗口 / -Report 时生成"],
        ["项目", "mapping/scripts/obstacle_layer", "实时描黑（-LiveSolid 才开）：/map -> /map_solid；solidify() 也给 solidify_map 用"],
        ["项目", "mapping/scripts/vote_map", "两种投票地图（默认开，-NoVote 关）：/scan_pc_raw -> /map_vote_raw（只投票）、/scan_pc -> /map_vote（投票 + 地面剔除），按位姿图优化实时重投；保存时存 map_vote_raw.*、map_vote.*"],
        ["项目", "mapping/config/mapper.yaml、mapper_native.yaml", "SLAM 参数（只差两个阈值参数；11-03 匹配 / 回环改回默认）"],
        ["项目", "mapping/rviz/slam_mapping.rviz", "建图 rviz 配置（含 Map Solid、Map Vote Raw、Map Vote 三层，后两层默认不打勾）"],
        ["D:\\ubuntu\\slam", "run_slam_windows.ps1 / .bat、pixi.toml（11-04 加 robot_localization）、fastdds_car_nic.xml（自动生成）", "Windows 一键：建图 / -Check / -Save / -Report / -Fast / -Odom ekf"],
        ["D:\\ubuntu\\slam", "maps\\run_*\\", "地图输出"],
        ["备份", "windows建图linux建图分布式通信\\", "源码副本、模拟测试脚本"],
    ]),
])

# ------------------------------------------------------------------ 第 4 章
UPLOAD = dict(title="第 4 章  上传到小车：车端 launch、里程计 /key1_odom 与 Run_car_safe", intro="11-04 快捷键 1 的建图里程计改名 /key1_odom；跑路径 / 录路径代码保持原样（不用动）。用 git 同步（第 2 章）+ 车上编译。Run_car_safe.sh 不在 install 里，仍用 scp。", blocks=[
    ("table", ["改动", "在哪运行", "要传小车吗"], [
        ["mapping/launch/distributed_mapping_vehicle.launch.py（快捷键 1：只起 key1_imu_encoder_mix，输出 /key1_odom；11-04 删掉 scan_relay 和录包）", "小车", "要"],
        ["跑路径 / 录路径（move_test 等 5 个 launch、record_seg.py）", "小车", "不改（11-04 恢复原样，照旧发 / 订 /encoder_imu_odom）"],
        ["CMakeLists.txt（去掉 scan_relay，加 ekf_input）；删除 mapping/scripts/scan_relay", "小车", "要（随 git 一起）"],
        ["Run_car_safe.sh（启动前清残留、退出时清整组进程）", "小车", "要（scp）"],
        ["scan_tf_bridge、ekf_input、map_report、mapper*.yaml、ekf.yaml、run_slam_windows.ps1 等", "电脑", "不用（随 git 到车上也不影响）"],
    ], [6, 1.2, 2.8]),
    ("warn", "小车上有没提交的改动（move_test.launch.py、record_seg.py、get_local_path.cpp、path.csv 等），直接快进合并会被拒。先在小车 git status 看清楚，提交或备份，不要 checkout -- / stash / reset --hard（见第 2 章、《改进与问题记录》一）。"),
    ("p", "小车上别人的路径代码一直在更新，所以分四步：① 小车先把别人的改动提交保存并打标签；② 电脑只提交建图相关的文件（mapping/、CMakeLists.txt）；③ 电脑拉小车 main 合并（合并在电脑上做，有冲突也在电脑上解决）；④ 小车快进到合并结果。小车上的路径代码一个字都不会变。"),
    ("h2", "4.1 小车：先保存别人的改动"),
    ("code", r"""# 小车
cd /home/racecar/src/my_racecar
git status --short
git add -A
git commit -m "小车：路径代码更新（上传建图代码前保存）"
git tag before-pc-20261104
git log --oneline -3"""),
    ("p", "git status 什么都不打印时，git commit 会说 nothing to commit，没关系，接着往下。正常：最后一行列出 3 个提交，最上面是刚才那条（或车上原来最新那条）。"),
    ("h2", "4.2 电脑：提交建图改动、合并小车代码、推上去"),
    ("code", r"""# 电脑 PowerShell（要输入小车 SSH 密码；CLAUDE.md 不提交）
cd "C:\Users\lzc2\Desktop\my_racecar（复件）"
git add -A -- mapping CMakeLists.txt
git update-index --chmod=+x mapping/scripts/ekf_input mapping/scripts/fit_track mapping/scripts/map_report mapping/scripts/solidify_map
git commit -m "建图：建图里程计改名 /key1_odom、删抗丢包、去畸变、EKF、赛道拟合、存图后描黑和报告"
git fetch car
git merge car/main -m "合并小车路径代码"
git push car main:from-pc"""),
    ("p", "正常：merge 打印 Merge made by 或 Already up to date，push 最后一行 main -> from-pc。merge 报 CONFLICT（多半是 CMakeLists.txt，两边都加了安装行）：先别推，把冲突文件名发给 AI 处理。"),
    ("h2", "4.3 小车：快进并编译"),
    ("code", r"""# 小车
cd /home/racecar/src/my_racecar
git merge --ff-only from-pc
git diff --stat before-pc-20261104 HEAD
cd /home/racecar
source /opt/ros/humble/setup.bash
colcon build --packages-select my_racecar"""),
    ("p", "正常：merge 打印 Fast-forward；git diff --stat 只列 mapping/... 和 CMakeLists.txt（说明路径代码没变）；colcon 最后一行 Summary: 1 package finished。merge 报 “Not possible to fast-forward”：4.1 之后车上又有人提交了，重做 4.1（commit、不用再打标签）和 4.2 的 fetch / merge / push。要退回：git reset --hard before-pc-20261104（车上没有没提交的改动时才能用）。"),
    ("code", r"""# 电脑 PowerShell：Run_car_safe.sh（要输入小车 SSH 密码）
scp "C:\Users\lzc2\Desktop\my_racecar（复件）\scripts\Run_car_safe.sh" "root@192.168.5.100:/home/racecar/Run_car_safe.sh"
"""),
    ("code", r"""# 小车
sed -i 's/\r$//' /home/racecar/Run_car_safe.sh
chmod +x /home/racecar/Run_car_safe.sh
bash -n /home/racecar/Run_car_safe.sh && echo "Run_car_safe.sh 语法 OK" """),
    ("h2", "4.4 日常启动顺序"),
    ("ul", [
        "小车 SSH 窗口一：bash /home/racecar/Run_car_safe.sh（取代 bash Run_car.sh，原脚本仍可用）。",
        "等约 10 秒，小车 SSH 窗口二：建图时敲 1（启动建图专用里程计，发 /key1_odom），只敲一次。跑路径、录路径照平时，不用敲 1。",
        "电脑：敲 2（轮速 + IMU 位姿对照）或 4（原高速激光 SLAM，见 3.6）开始建图，两者单独运行；建完另开窗口敲 a / b / c / d（按抽中的赛道）或 3（自动选）保存，在这个保存窗口里看到“第 1 趟建图已完整保存”再关建图窗口（详细见 3.2）。",
    ]),
    ("p", "首次试用 Run_car_safe.sh 时确认 Ctrl+C 能清干净：见《改进与问题记录》三、上传到小车。上车后确认只有一个里程计：见《改进与问题记录》二“两个里程计同时在发”。"),
    ("h2", "4.5 回滚"),
    ("p", "按 4.3 的 before-pc-20261104 标签（或第 2 章 2.4 合并前打的标签）回退；停用安全版：直接 bash Run_car.sh 即可（原文件没动）。"),
    ("pkg", [
        ["项目", "scripts/Run_car_safe.sh", "安全版底盘驱动启动脚本"],
        ["小车", "Run_car_safe.sh", "上传后的位置"],
        ["项目", "mapping/launch/distributed_mapping_vehicle.launch.py", "快捷键 1：建图专用里程计 /key1_odom"],
        ["备份", "上传到小车\\", "文件副本"],
    ]),
])

# ------------------------------------------------------------------ 第 5 章
CSV = dict(title="第 5 章  CSV 路径与地图融合查看", intro="把已保存的地图和 CSV 路径放进同一个 rviz2 里看。离线运行，不需要小车，不做拟合。", blocks=[
    ("h2", "5.1 最省事：拖拽"),
    ("ul", [
        "把一个或多个 .csv 拖到 临时修改\\csv路径和slam融合\\拖拽CSV到这里.bat 上松手，自动弹出 rviz2；关掉 rviz2 窗口即退出。",
        "双击（不拖文件）：只显示地图 + 项目里的 path.csv。",
        "固定地图：同文件夹 run_20261005_1552_filtered\\run1\\map.yaml；换地图改 drop_view.ps1 开头的 $mapDirName。",
        "每个 CSV 一种颜色，起点绿球、终点红球，标文件名和点数；改文件 1 秒内自动刷新。",
        "get_local_path 的跟踪日志（path_tracking_*.csv，表头 px,py,...,cx,cy,...）自动拆成“目标”“实际”两条线。",
    ]),
    ("code", r"""# 电脑 PowerShell：命令行等价写法
cd "D:\VSCode\ros2技术集\智能车总文件\my_racecar（复件）\事项\临时修改\csv路径和slam融合"
.\drop_view.ps1 "D:\a.csv" "D:\b.csv"
"""),
    ("h2", "5.2 Windows：view_csv（D:\\ubuntu\\slam）"),
    ("code", r"""# 电脑 PowerShell：最新保存的地图 + 调试 CSV
cd D:\ubuntu\slam
$m = (Get-ChildItem D:\ubuntu\slam\maps -Recurse -Filter map.yaml | Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName
Write-Host "地图: $m"
.\view_csv.bat -Map $m -Extra "D:\debug\a.csv","D:\debug\b.csv"
"""),
    ("table", ["参数", "作用"], [
        ["-Map <map.yaml>", "叠加地图；不给就只看路径"],
        ["-Extra a.csv,b.csv", "调试 CSV，可多个，改文件自动刷新"],
        ["-NoPath", "不画 path.csv，只画 -Extra"],
        ["-Inflate 1", "障碍格加粗 1 格（只影响显示，地图文件不变），小锥桶更醒目"],
        ["-Dx 0.2 -Dy -0.1 -Dyaw 0.05", "路径和地图对不上时手动平移 / 旋转（米、弧度）"],
        ["-AbA ab_a.csv -ForkIndex -1", "显示 A/B 停车轨迹（只给 A，B 自动镜像）；-ForkIndex 是分叉点在 path.csv 的序号"],
        ["-PathCsv <path.csv>", "换路径文件"],
    ], [3, 6]),
    ("h2", "5.3 Linux 电脑"),
    ("code", r"""# Linux 电脑（先 colcon build --packages-select my_racecar）
cd "/data/code/智能车/智能车(1)/my_racecar（复件）(1)"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
MAP=$(ls -dt ~/maps/distributed_*/run1/map.yaml 2>/dev/null | head -n 1)
echo "地图: ${MAP:-无}"
ros2 launch my_racecar view_path_on_map.launch.py ${MAP:+map_yaml:=$MAP} extra_csv:=/路径/debug_a.csv,/路径/debug_b.csv"""),
    ("h2", "5.4 CSV 格式"),
    ("ul", [
        "有表头：按列名 x、y 取（顺序随意，有 yaw 也读）；无表头：取前两列。空行和 # 注释行跳过。",
        "坐标必须和地图同一坐标系（以建图起点为原点）。录 path.csv 和建图时车要从同一位置、同一朝向出发，否则整体偏移，用 -Dx/-Dy/-Dyaw 调。",
        "起点相对坐标的文件（ab_a.csv，从 0,0 开始）通过 -AbA 传，会按分叉点自动摆放。",
    ]),
    ("pkg", [
        ["项目", "mapping/scripts/view_path_on_map", "节点：读 pgm/yaml 发地图，读 CSV 发路径、调试段、分叉点、静态 TF"],
        ["项目", "mapping/launch/view_path_on_map.launch.py", "Linux 入口（节点 + rviz2）"],
        ["项目", "mapping/rviz/path_on_map.rviz", "显示配置：path.csv 绿、A 橙、B 蓝、分叉点洋红"],
        ["项目", "include/path.csv、ab_a.csv、ab_b.csv", "路径文件"],
        ["D:\\ubuntu\\slam", "view_csv.ps1 / .bat", "Windows 入口（项目路径写死在 ps1 开头的 $proj）"],
        ["备份", "csv路径和slam融合\\拖拽CSV到这里.bat、drop_view.ps1", "拖拽入口"],
    ]),
])

# ------------------------------------------------------------------ 第 6 章
AB = dict(title="第 6 章  AB 路径拼接", intro="从发车区跑 2 圈，第 2 圈快结束时拐进 AB 停车区，停在 A 或 B 结束。完整路径和 AB 轨迹分开录，启动时指定停哪个。以下都在小车上。", blocks=[
    ("h2", "6.1 编译（拷入改动文件后做一次）"),
    ("code", r"""# 小车
cd /home/racecar
source /opt/ros/humble/setup.bash
colcon build --packages-select my_racecar --symlink-install
source /home/racecar/install/local_setup.bash"""),
    ("h2", "6.2 录完整路径（发车区出发跑两圈，到分叉点停）"),
    ("code", r"""# 小车：save_path 会覆盖 path.csv，先自动备份
cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
export ROS_DOMAIN_ID=42
cp /home/racecar/src/my_racecar/include/path.csv /home/racecar/src/my_racecar/include/path.csv.bak_$(date +%Y%m%d_%H%M%S) 2>/dev/null
ros2 launch my_racecar save_path.launch.py"""),
    ("p", "到分叉点后 Ctrl+C。"),
    ("h2", "6.3 录 A 停车轨迹（车放到分叉点，车头沿路径方向）"),
    ("code", r"""# 小车
cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
export ROS_DOMAIN_ID=42
ros2 launch my_racecar save_ab_path.launch.py ab:=a"""),
    ("p", "开到 A 停车位后在这个终端按 Ctrl+C，生成 include/ab_a.csv。B 不用录，自动镜像；想单独录 B 用 ab:=b。确认录上了：在小车上运行下面这条，文件存在、行数几十行以上（每 0.1 m 一点）、修改时间是刚才："),
    ("code", r"""# 小车：看 A 停车轨迹文件
ls -l /home/racecar/src/my_racecar/include/ab_a.csv && wc -l /home/racecar/src/my_racecar/include/ab_a.csv && head -3 /home/racecar/src/my_racecar/include/ab_a.csv"""),
    ("warn", "AB 录制起点必须和 path.csv 末点（分叉点）的位置、朝向一致，否则 AB 段整体偏。重录 path.csv 后 AB 也要重录。"),
    ("h2", "6.4 比赛：启动时指定停 A 或 B"),
    ("code", r"""# 小车
cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
export ROS_DOMAIN_ID=42
ros2 launch my_racecar move_test.launch.py ab:=a"""),
    ("p", "停 B 用 ab:=b；不拼 AB 用 ab:=none。启动后就在这个终端往上翻启动日志，会打印 AB 轨迹来源（CSV / CSV（另一侧镜像）/ 代码内置）和分叉点序号。"),
    ("h2", "6.5 get_local_path 新增参数"),
    ("table", ["参数", "默认", "说明"], [
        ["ab", "launch 里默认 a", "a / b / none，覆盖原来的 ab_flag"],
        ["ab_speed", "1.5", "AB 段速度；0 = 用 CSV 第 4 列"],
        ["ab_fork_index", "-1", "分叉点在 path.csv 的序号，负数从末尾倒数；多录了一段可用 -20 截掉"],
        ["laps", "1", "path.csv 原样用；只录了 1 圈时设 2，自动重复一圈"],
        ["ab_a_csv / ab_b_csv", "include/ab_a.csv、ab_b.csv", "AB 轨迹文件"],
    ], [2.2, 2.4, 5.5]),
    ("note", "pure_track_test 的 first_stop_index / second_stop_index / traffic_light_index 按拼接后的下标计数，要用时按实际路径长度重算。运行中也可以向 /ab_select 发 a / b / none 切换（一般用不到）。"),
    ("warn", "这些改动目前只在 临时修改\\AB路径\\ 里，还没拷进项目（2026-10-17 核对：项目里没有 save_ab_path.cpp）。用之前先把文件放进项目对应位置，再按 6.1 编译。"),
    ("pkg", [
        ["备份", "AB路径\\src\\save_ab_path.cpp → 项目 src/", "新增：AB 轨迹录制节点（起点相对坐标，每 0.1 m 一点）"],
        ["备份", "AB路径\\launch\\save_ab_path.launch.py → 项目 launch/", "新增：里程计 + 录制，ab:=a / b"],
        ["备份", "AB路径\\src\\get_local_path.cpp → 项目 src/", "修改 7 处（搜【AB改动），拼接、镜像、去重；副本已转 UTF-8（原项目是 GBK）"],
        ["备份", "AB路径\\launch\\move_test.launch.py → 项目 launch/", "修改：新增 ab 参数"],
        ["备份", "AB路径\\CMakeLists.txt → 项目根", "注册 save_ab_path"],
    ]),
])

# ------------------------------------------------------------------ 第 7 章
VLOCAL = dict(title="第 7 章  红绿灯识别：小车本地（.om）", intro="小车 NPU 跑 .om：light_infer.py（conda python3）写状态文件，light_bridge.py（/usr/bin/python3）发话题。和第 8 章的远程识别不能同时运行（共用摄像头和话题名）。", blocks=[
    ("h2", "7.1 第一次部署：拷模型"),
    ("code", r"""# 电脑 PowerShell：.onnx 和 .om 不进 git，单独拷
ssh root@192.168.5.100 "mkdir -p /home/racecar/src/my_racecar/camera_capture/output"
scp "D:\VSCode\ros2技术集\智能车总文件\my_racecar（复件）\事项\临时修改\yolov5n11n.om训练过程\06_模型文件\yolov5n_bs1.om" root@192.168.5.100:/home/racecar/src/my_racecar/camera_capture/output/"""),
    ("p", "文件名和位置必须是 camera_capture/output/yolov5n_bs1.om。转换方法见第 9 章。"),
    ("h2", "7.2 运行：识别 + 发布话题"),
    ("code", r"""# 小车终端 1（先停掉远程识别）
cd /home/racecar/src/my_racecar/camera_capture && bash start_vision.sh"""),
    ("code", r"""# 小车终端 2：看结果
source /opt/ros/humble/setup.bash && /usr/bin/python3 /home/racecar/src/my_racecar/camera_capture/light_listener.py"""),
    ("p", "终端 2 每 2 秒打印一行（状态变了立即多打一行），样子如下，数字是示例（“状态=”后面就是识别结果；green 才是 GO）："),
    ("code", r"""# 小车终端 2 里自动刷出的（不是要敲的命令）
[INFO] [light_listener]: [20Hz] 状态=red STOP | 灯箱 (301,122)-(352,290) 高168px 置信度0.91 亮灯0.88 延迟45ms 推理18.0fps
[INFO] [light_listener]: 状态变化：red -> green
[INFO] [light_listener]: [20Hz] 状态=green GO | 没有灯箱"""),
    ("p", "前面的 [20Hz] 应接近 20；一直是 0 或只打印“等待 /traffic_state ...”说明终端 1 没在发，看终端 1 的报错。"),
    ("ul", [
        "start_vision.sh 后台启动 light_infer.py，等到第一帧结果，再前台启动 light_bridge.py；Ctrl+C 同时停两个。识别进程的日志在 camera_capture/infer.log，另开小车终端运行 tail -f /home/racecar/src/my_racecar/camera_capture/infer.log 实时看。",
        "启动前会检查 .om 是否存在、摄像头是否被占用、light_infer.py 是否已在运行。",
        "带参数：bash start_vision.sh --conf-box 0.4 --conf-lamp 0.4（传给 light_infer.py）。",
    ]),
    ("table", ["话题", "类型", "内容"], [
        ["/traffic_state", "std_msgs/String", "red / green / off / none / conflict / stale；20 Hz"],
        ["/traffic_light_box", "Float32MultiArray", "[x1,y1,x2,y2,灯箱置信度,亮灯置信度,图宽,图高,结果延迟秒,推理fps]，没灯箱时框为 -1"],
        ["/traffic_light", "Int32", "1 = 绿灯放行。电脑项目版本里关闭（标了 [控制]）"],
    ], [2.2, 2.4, 5.5]),
    ("note", "只有明确 green 才放行；stale（状态文件超过 0.5 秒没更新）、off、none、conflict 都不放行。"),
    ("h2", "7.3 常用参数（light_infer.py）"),
    ("table", ["参数", "说明"], [
        ["--model / --source", ".om 路径；摄像头编号、视频或图片文件夹"],
        ["--conf-box / --conf-lamp", "灯箱、亮灯置信度阈值"],
        ["--confirm", "连续几帧一致才切换状态（默认 3）"],
        ["--state-file", "状态文件，默认 /tmp/light_state.txt"],
        ["--ar-min/--ar-max、--cy-min/--cy-max、--no-filter", "几何过滤（电脑项目版本），以 --help 为准"],
    ], [3.5, 6]),
    ("h2", "7.4 停止"),
    ("p", "终端 1 按 Ctrl+C。确认摄像头释放、只调识别不发话题（run.sh）等调试命令见《改进与问题记录》八、调试命令。"),
    ("warn", "小车上的 light_bridge.py、start_vision.sh 有未提交改动，还多了 light_infer_daemon.py，没有同步到电脑；小车实际行为以车上文件为准。"),
    ("pkg", [
        ["小车项目", "camera_capture/start_vision.sh", "一键：识别 + 发话题"],
        ["小车项目", "camera_capture/light_infer.py、det_utils.py", "识别和判断（conda python3）"],
        ["小车项目", "camera_capture/light_bridge.py", "读状态文件发话题（/usr/bin/python3）"],
        ["小车项目", "camera_capture/light_listener.py", "订阅打印（接收端示例）"],
        ["小车项目", "camera_capture/output/yolov5n_bs1.om", "模型（不进 git）"],
        ["小车项目", "camera_capture/run.sh、check_env.sh、convert.sh、record_video.py", "调试识别、环境检查、车上转 om、录视频"],
    ]),
])

# ------------------------------------------------------------------ 第 8 章
VREMOTE = dict(title="第 8 章  红绿灯识别：电脑远程", intro="小车只拍摄、JPEG 编码、转发，电脑用 YOLOv5n（onnx）识别，结果传回小车发布同样的话题。小车上不需要 conda、CANN、NPU。", blocks=[
    ("h2", "8.1 启动（验证模式，不影响运动）"),
    ("code", r"""# 小车（先停掉本地识别 start_vision.sh）
cd /home/racecar/src/my_racecar/camera_capture && bash start_remote_vision.sh"""),
    ("p", "就在这个小车终端里看到“等待电脑连接 0.0.0.0:9000”就是就绪；电脑端连上后这里会打印连接信息。建议只监听小车 WiFi 地址：BIND=192.168.5.100 bash start_remote_vision.sh。"),
    ("code", r"""# 电脑 PowerShell
cd "C:\Users\lzc2\Desktop\my_racecar（复件）\camera_capture"
powershell -ExecutionPolicy Bypass -File .\start_pc_vision.ps1"""),
    ("p", "不弹窗加 -NoShow；指定地址 -Car 192.168.5.100 -Port 9000；窗口里按 q 退出。不用脚本直接运行："),
    ("code", r"""# 电脑 PowerShell
cd "C:\Users\lzc2\Desktop\my_racecar（复件）\camera_capture"
D:\light_ai\venv\Scripts\python.exe pc_vision_client.py --car 192.168.5.100"""),
    ("code", r"""# 小车另开终端：看结果
source /opt/ros/humble/setup.bash && /usr/bin/python3 /home/racecar/src/my_racecar/camera_capture/light_listener.py"""),
    ("p", "输出和 7.2 一样：每 2 秒一行“[20Hz] 状态=… | 灯箱 …”；电脑端不弹窗时就靠这里看结果，弹窗时电脑窗口里也画框。"),
    ("h2", "8.2 接入运动控制（联动 pure_track_test 时才用）"),
    ("code", r"""# 小车
cd /home/racecar/src/my_racecar/camera_capture && bash start_remote_vision.sh --enable-control"""),
    ("ul", [
        "加 --enable-control：订阅 /traffic_light_req，收到 1 期间才发 /traffic_light（green 发 1，其他含 stale 发 0）；收到 0 停止发布。",
        "不加：只发 /traffic_state、/traffic_light_box，不影响运动。",
        "运动侧要用 light_mode: 1、traffic_light_index（现在 258，需现场标定）、traffic_light_time（10 秒，超时放行）。",
        "第一次接入建议架空轮子；改了 C++ 要自己 colcon build。这条 ROS 路径只验证过语法，没在车上跑过。",
    ]),
    ("h2", "8.3 参数"),
    ("table", ["小车端参数", "默认", "说明"], [
        ["--fps / --quality", "15 / 80", "发给电脑的帧率上限、JPEG 质量；负载高降到 10 / 70"],
        ["--stale-sec", "0.5", "结果距拍摄超过这么久就当 stale；WiFi 抖动可放宽到 0.8–1.0"],
        ["--port / --rate", "9000 / 20", "监听端口；话题发布频率"],
        ["--enable-control", "关", "见 8.2"],
        ["--selftest", "关", "只测采集编码负载，不连电脑"],
        ["BIND（环境变量）", "0.0.0.0", "监听地址，建议 192.168.5.100"],
    ], [2.6, 1.6, 5.5]),
    ("table", ["电脑端参数", "默认", "说明"], [
        ["--car / --port", "192.168.5.100 / 9000", "小车地址"],
        ["--model", "视觉模型\\模型\\yolov5n_best.onnx", "换模型"],
        ["--conf-box / --conf-lamp / --confirm", "0.5 / 0.5 / 3", "阈值和确认帧数"],
        ["--ar-min/--ar-max、--cy-min/--cy-max", "2.5–4.5、0.20–0.85", "灯箱几何过滤；--no-filter 关闭"],
        ["--no-show", "关", "不弹窗"],
    ], [3.2, 2.6, 4]),
    ("h2", "8.4 停止与切换"),
    ("ul", [
        "电脑端窗口按 q 或 Ctrl+C；小车端 Ctrl+C。电脑重开会自动重连。",
        "和第 7 章切换：先 Ctrl+C 停掉当前这套。start_remote_vision.sh 发现车上识别在运行或摄像头被占用会拒绝启动。",
        "总延迟约 100–400 ms，车速 1–2 m/s 会多滑 0.1–0.8 m，停车阈值要算进去。TCP 端口没有认证，只在局域网用。比赛是否允许场外计算赛前要确认。",
    ]),
    ("pkg", [
        ["项目", "camera_capture/remote_vision_bridge.py", "小车：采集 + 编码 + TCP 服务端 + 发话题 + 看门狗（/usr/bin/python3）"],
        ["项目", "camera_capture/start_remote_vision.sh", "小车入口（检查摄像头占用）"],
        ["项目", "camera_capture/remote_proto.py", "两边共用的通信协议"],
        ["项目", "camera_capture/pc_vision_client.py", "电脑：收帧、识别、回传、显示（复用 light_infer.py）"],
        ["项目", "camera_capture/start_pc_vision.ps1", "电脑入口（先检查端口）"],
        ["电脑", "D:\\light_ai\\venv", "onnxruntime、torch、opencv"],
    ]),
])

# ------------------------------------------------------------------ 第 9 章
TRAIN = dict(title="第 9 章  红绿灯模型：训练、导出、转 .om", intro="采集 → 标注 → 训练 YOLOv5n → 导出 ONNX（opset 12）→ Linux 电脑转 .om → 小车加载。三类：0 light_box、1 red_on、2 green_on。", blocks=[
    ("h2", "9.1 采集与抽帧"),
    ("code", r"""# 小车：录视频（摄像头装在最终位置；用 /usr/bin/python3）
cd /home/racecar/src/my_racecar/camera_capture
/usr/bin/python3 record_video.py --name green_near --seconds 20"""),
    ("code", r"""# 电脑 PowerShell：拷回、抽帧（每 5 帧一张）
scp root@192.168.5.100:/home/racecar/src/my_racecar/camera_capture/videos/*.mp4 "D:\VSCode\ros2技术集\智能车总文件\视觉模型\数据\视频\"
python extract_frames.py D:\light_ai\data\videos\某视频.mp4 --every 5"""),
    ("p", "标注、划分、打包用 02_标注与数据集\\ 里的脚本（里面写的是当时的路径 D:\\light_ai\\...，重跑要改路径），得到 light_dataset.zip。"),
    ("h2", "9.2 训练 YOLOv5n（有显卡的 Linux 电脑）"),
    ("code", r"""# Linux 训练电脑
git clone https://github.com/ultralytics/yolov5 && cd yolov5 && git checkout v7.0
pip install -r requirements.txt && pip install onnx
python train.py --img 640 --batch 16 --epochs 100 --data light.yaml --weights yolov5n.pt
python export.py --weights runs/train/exp/weights/best.pt --include onnx --imgsz 640 --opset 12"""),
    ("warn", "必须 --opset 12，不要 --dynamic。上车只能用 YOLOv5 v7.0（课程的推理按它的输出格式 [1,25200,8] 写），YOLO11n 只能在电脑上用。"),
    ("h2", "9.3 电脑上检查和验证"),
    ("code", r"""# 电脑 PowerShell（在 yolov5n11n.om训练过程\ 下）
D:\light_ai\venv\Scripts\python.exe 04_转om\check_onnx.py 06_模型文件\yolov5n_best.onnx
D:\light_ai\venv\Scripts\python.exe 05_验证\verify_val.py --model 06_模型文件\yolov5n_best.onnx"""),
    ("p", "验证集 344 张：错 0 张，红判绿 0（验证集和训练集同一间教室、同一块灯箱，分数偏高）。"),
    ("h2", "9.4 Linux 电脑转 .om（CANN 7.0.RC1 x86_64）"),
    ("code", r"""# Linux 电脑（onnx 改名 yolov5n.onnx，放英文路径）
source ~/Ascend/ascend-toolkit/set_env.sh
atc --model=yolov5n.onnx --framework=5 --output=yolov5n_bs1 --input_format=NCHW \
    --input_shape="images:1,3,640,640" --log=error --soc_version=Ascend310B4 \
    --input_fp16_nodes=images --output_type=FP16
ls -lh yolov5n_bs1.om"""),
    ("p", "电脑上的 CANN 必须也是 7.0.RC1（innerversion 最好也是 V100R001C13SPC005B246）；soc 写小车的 Ascend310B4。装 CANN 的步骤：Python 3.9 环境、pip 装依赖、下载 Ascend-cann-toolkit_7.0.RC1_linux-x86_64.run 后 --install。"),
    ("h2", "9.5 拷到小车并加载"),
    ("code", r"""# 电脑 / Linux 电脑
scp yolov5n_bs1.om root@192.168.5.100:/home/racecar/src/my_racecar/camera_capture/output/

# 小车（root）
cd /home/racecar/src/my_racecar/camera_capture && source /usr/local/Ascend/ascend-toolkit/set_env.sh
python3 -c "from ais_bench.infer.interface import InferSession; s=InferSession(0,'output/yolov5n_bs1.om'); print('load ok')"
bash start_vision.sh"""),
    ("p", "打印 load ok 说明 .om 和车上 CANN 兼容（已实测）。"),
    ("pkg", [
        ["备份", "yolov5n11n.om训练过程\\01_采集与抽帧\\", "record_video.py、extract_frames.py"],
        ["备份", "yolov5n11n.om训练过程\\02_标注与数据集\\", "自动标注、合并、剔除、打包脚本"],
        ["备份", "yolov5n11n.om训练过程\\03_训练\\", "light.yaml、两个模型的结果曲线和 results.csv"],
        ["备份", "yolov5n11n.om训练过程\\04_转om\\", "check_onnx.py、check_om.py、课程转换工程"],
        ["备份", "yolov5n11n.om训练过程\\05_验证\\", "verify_val.py、light_yolo11_cam.py"],
        ["备份", "yolov5n11n.om训练过程\\06_模型文件\\", "yolov5n_best.onnx/.pt、yolov5n_bs1.om、yolo11n_best.pt"],
        ["电脑", "D:\\VSCode\\ros2技术集\\智能车总文件\\视觉模型\\数据\\数据集打包_给训练电脑用\\light_dataset.zip", "数据集（约 80 MB，1086 张）"],
    ]),
])

# ------------------------------------------------------------------ 附录
APPX = dict(title="附录  路径速查", intro="配套文件表里“位置”一列对应的实际路径。", blocks=[
    ("paths",),
    ("h2", "其他常用位置"),
    ("table", ["内容", "路径"], [
        ["Windows 建图地图", "D:\\ubuntu\\slam\\maps\\run_<日期_时分>_<模式>\\run1\\map.png"],
        ["Linux 建图地图", "~/maps/distributed_<时间>/run1/"],
        ["建图实时日志", "<地图目录>\\.run1-live.jsonl（每 5 秒一行）"],
        ["view_csv 节点日志", "%TEMP%\\view_csv_node.err"],
        ["视觉模型", "D:\\VSCode\\ros2技术集\\智能车总文件\\视觉模型\\模型\\yolov5n_best.onnx"],
        ["本文档源文件", "临时修改\\文档\\源\\（manual.py 等，改完运行 build.py）"],
        ["旧 md（只读）", "临时修改\\旧md归档\\"],
    ], [2.5, 7.5]),
])

CHAPTERS = [ENV, GIT, MAP, UPLOAD, CSV, AB, VLOCAL, VREMOTE, TRAIN, APPX]
