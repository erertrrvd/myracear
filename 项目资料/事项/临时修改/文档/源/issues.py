# -*- coding: utf-8 -*-
"""《改进与问题记录》内容。每个问题一张卡片：现象 / 排查 / 原因 / 解决 / 结果。
新问题追加在对应模块章节的末尾（按发生顺序）；调试、验证命令放最后一章“八、调试命令”。
"""
UPDATED = "2026-10-22"

C = lambda s: ("code", s)
HEAD = r"""cd /home/racecar
source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0"""

# ------------------------------------------------------------------ 一、git
GIT = dict(title="一、git 同步", intro="电脑和小车同步时遇到的报错。最常见的原因是命令在错误的终端里运行。", blocks=[
    ("table", ["现象", "原因", "处理"], [
        ["untracked working tree files would be overwritten by merge", "小车上有未跟踪的同名文件（手动拷上去的）", "先挪走再合并，不删除：mkdir -p /root/pc_vision_backup && mv 那几个文件 /root/pc_vision_backup/，再 git merge --ff-only from-pc"],
        ["Your local changes ... would be overwritten by merge", "小车上已跟踪文件有未提交改动，合并要改同一个文件", "先在小车提交，或 git diff > 文件.patch 备份再合并。不要用 checkout --、stash"],
        ["fatal: Not possible to fast-forward, aborting", "两边都有新提交，分叉了", "电脑先 git fetch car、git merge car/main，再推 from-pc"],
        ["refusing to update checked out branch", "往小车当前分支 main 推了", "一律 git push car main:from-pc"],
        ["fatal: could not open '?' for reading", "在 PowerShell 里运行了 xargs", "改用 Git Bash"],
        ["Already : 无法将“Already”项识别为 cmdlet", "把输出 Already up to date. 当命令粘进去了", "只粘命令，不粘输出；没有影响"],
        ["Permission denied, please try again.", "SSH 密码输错", "重输；终端里输密码不显示字符"],
        ["输出停在 :", "分页器", "按 q；以后用 git --no-pager"],
        ["中文注释显示成 ▒ / <B6>", "GBK 文件按 UTF-8 显示", "只是显示问题；用补丁和 git 传输，不要复制粘贴"],
        ["合并后小车行为没变", "没有重新编译", "自己决定是否 colcon build"],
    ], [3.2, 3, 4.2]),
    ("issue", "scp 到电脑的命令在小车终端里运行了", {
        "现象": "取回小车视觉改动时，scp ... \"$env:USERPROFILE\\...\" 在小车终端里执行，补丁没拷到电脑。",
        "原因": "$env:USERPROFILE 是 PowerShell 的写法，scp 到电脑必须在电脑 PowerShell 里运行。",
        "排查": [C("# 小车：检查有没有留下杂文件\ncd /home/racecar/src/my_racecar && git status --short; ls -la /root /home/racecar/src/my_racecar | grep -i userprofile")],
        "解决": "有杂文件就贴出来确认后再删；补丁还在小车 /tmp/，在电脑 PowerShell 里重新 scp。",
    }),
    ("h2", "回退（破坏性，先备份）"),
    ("code", r"""# 小车：先备份未提交的视觉文件
mkdir -p /root/backup_before_reset && cp camera_capture/light_bridge.py camera_capture/start_vision.sh camera_capture/light_infer_daemon.py /root/backup_before_reset/
# 确认后才回到某个提交（例如小车自己的红绿灯提交 02a512a）
git reset --hard 02a512a"""),
])

# ------------------------------------------------------------------ 二、分布式建图
MAP = dict(title="二、分布式建图", intro="按发生顺序：Linux 上地图为空 → Windows 收不到数据 → 地图质量 → 漂移。", blocks=[
    ("issue", "11-10 17:29 建图出现大面积黑点和地图错位", {
        "现象": "RViz 中散布黑色圆盘，地图边缘呈放射状，雷达红线与地图障碍存在偏离。截图需要与具体运行时刻对应，不能仅靠截图判断地图中的实际障碍。",
        "排查": ["电脑 D:\\ubuntu\\slam\\maps\\run_20261110_1729_fast\\.run1-live.jsonl：17:31:35 的 map→odom_pc 修正相对上一个 5 秒记录变化约 5.32 m / -58.6°，17:31:40 又变化约 3.97 m / -55.8°；SLAM 与里程计明显分歧。",
                 "电脑 C:\\Users\\lzc2\\.ros\\log\\python_11632_1794303027150.log：地面线全程未认出；突跳附近等待里程计没有超时，不能据此断言地面过滤误删或 WiFi 延迟是根因。",
                 "电脑 C:\\Users\\lzc2\\.ros\\log\\python_10460_1794303026470.log：cone_marks 从 /map 中识别点团并显示黑圆盘，数量最高记录 117 个；它只画标记，不修改 SLAM 原图。"],
        "原因": "已确认 SLAM 位姿修正出现严重突跳，解释轨迹错位；触发突跳的具体原因尚不能凭状态日志唯一确定，仍需同趟原始扫描、里程计及场地实况。黑圆盘来自实时标记，不能凭标记认定是真锥桶。",
        "解决": "11-10 按用户要求删除实时锥桶描深节点、启动项和 RViz 图层；保存后的 solidify_map 描黑仍保留。保存原始图并继续对照原始包排查突跳。",
    }),
    ("issue", "点云能显示，但 map_cells=0，地图为空", {
        "现象": "电脑收到 /scan，rviz 有点云，slam_toolbox 刷 earlier than all the data in the transform cache / Message Filter dropping message，地图不出来。",
        "原因": "车端 /tf 经 WiFi 到达后时间戳对不上扫描（两端时钟也有偏差），slam_toolbox 把所有扫描丢掉。",
        "解决": "电脑端 scan_tf_bridge 不用车端 TF，按每帧扫描自己的时间戳发 odom_pc→base_pc（位姿取 /encoder_imu_odom）。不依赖两端时钟一致。",
        "结果": "Linux 电脑实车出图（提交 c587ccd）。",
    }),
    ("issue", "LaserRangeScan contains ... expected：扫描被拒收", {
        "现象": "slam_toolbox 报扫描点数不对，丢帧。",
        "原因": "雷达每帧点数不固定（366/367/368），slam_toolbox 拒收点数变化的帧。",
        "解决": "scan_tf_bridge 重采样成固定 720 点：按实际点数推角度，同角度格取最近点，用“反向取样”避免大量 inf。",
    }),
    ("issue", "墙外有放射状条纹", {
        "原因": "无回波的 inf 太多，射线穿墙把墙外擦成空闲。",
        "解决": "反向取样 + mapper.yaml 里 max_laser_range: 12。",
    }),
    ("issue", "/encoder_imu_odom 交替跳变", {
        "原因": "Run_car.sh 和建图入口各启动了一个 imu_encoder_mix。",
        "解决": "当时用 start_odom:=false；现在车端 launch 只启动 imu_encoder_mix，Run_car_safe.sh 启动前清残留。不要重复启动里程计（快捷键 1 不检查，别敲两次）。",
    }),
    ("issue", "Windows：rviz 没有点云、Map 宽高为 0（2026-10-04）", {
        "现象": ["run_slam_windows.ps1 启动后节点都没报错，但 rviz 里没有雷达点云，Map 显示 Width 0 / Height 0，只有中心一个小坐标轴。",
                 "坐标轴是 scan_tf_bridge 自己发的静态 TF，说明 rviz 和 TF 是通的；宽高为 0 = 一帧扫描都没收到。"],
        "排查": ["只读检查，没改系统设置：",
                 "网卡 5 块：WLAN（192.168.5.124）、以太网 2、Meta 代理隧道（198.18.0.1）、VMware VMnet1 / VMnet8。",
                 "到小车走 WLAN，ping 通；防火墙三种配置文件都是关的，不是防火墙。",
                 "默认配置下域 42、域 0 都发现不了小车任何节点；限定只走 WLAN 后立刻发现 14 个节点，收到 /scan（979 点）和 /encoder_imu_odom。",
                 "旧的 fastdds_wifi_only.xml 写死 192.168.5.121，和当前 IP（.124）不一致，不生效。"],
        "原因": "多余网卡（代理隧道、手机共享、虚拟网卡）干扰了 DDS 的发现流量，发现包没走到 WiFi 网卡。",
        "解决": "run_slam_windows.ps1 每次启动自动找出通往小车的网卡 IP，生成只走这块网卡的 fastdds_car_nic.xml 并启用。-Check 改成约 8 秒出结果。去掉 -WifiOnly，新增 -AllNics（一般不要用）。",
        "结果": "实车 -Check：发现 14 个节点，/scan 收到 979 点，/encoder_imu_odom 收到，/map 发布者 0。",
    }),
    ("issue", "-Check 选中了 Meta (198.18.0.1)，显示 DDS 没通", {
        "原因": "电脑当时连的是别的 WiFi（10.105.129.143/24），不在小车网段，去 192.168.5.100 的流量被默认路由送进代理隧道。",
        "解决": "选卡逻辑改为“找和小车同一网段的网卡”，找不到就红字提示没连小车 WiFi 并列出网卡，不再用路由判断。",
    }),
    ("issue", "13:14 那趟地图几乎不增长，障碍只有 25 格（2026-10-05）", {
        "原因": "小车没发 /encoder_imu_odom，位姿恒为原点；slam_toolbox 要移动超过 minimum_travel_distance（0.2 m）才处理新扫描。",
        "解决": ["scan_tf_bridge：开了里程计却收不到时报错；mapping_status_logger：日志加 known_cells，30 秒不增长写 stalled；",
                 "run_slam_windows.ps1：前台每 15 秒打印 [监控]，没里程计红字、不增长黄字；新增 map_autosave 每 60 秒自动存。"],
        "结果": "模拟验证通过；实车效果待验证。",
    }),
    ("issue", "地图里细锥桶很小或丢失", {
        "现象": "实车地图锥桶只剩几个点或没有。",
        "排查": ["模拟小车（979 点雷达、细锥桶 + 走动物体）对比参数 occupancy_threshold / min_pass_through："],
        "原因": "原阈值 0.35/4 在高速下把细锥桶一起过滤掉。",
        "解决": "mapper.yaml 改为 0.20/3；建图那一圈开慢（约 0.5 m/s）；看图加 -Inflate 1。另建 mapper_native.yaml（不设这两个参数）用于对比。",
    }),
    ("table", ["条件", "参数", "锥桶找回", "锥桶占据格", "走动物体残留"], [
        ["0.5 m/s，无畸变", "0.35 / 4（原）", "6/6", "39", "0"],
        ["", "0.20 / 3", "6/6", "56", "0"],
        ["", "0.10 / 2", "6/6", "73", "0"],
        ["1.5 m/s，带扫描畸变", "0.35 / 4（原）", "2/12", "6", "0"],
        ["", "0.20 / 3", "6/12", "19", "0"],
        ["", "0.10 / 2", "7/12", "24", "3"],
    ], [2.6, 2, 1.5, 1.6, 1.8]),
    ("note", "以上是模拟结果，实车要各建一张图对比。重采样点数 beams（720→1080）和去畸变 deskew 在实验里没有稳定收益，默认没改。"),
    ("issue", "ros2 launch 报 file ... was found more than once", {
        "原因": "install 里有指向已搬走源文件的悬空符号链接。",
        "解决": [C("# 小车（rm 只删链接本身，不碰源码；不需要重新编译）\ncd /home/racecar\nsource /opt/ros/humble/setup.bash\nfind /home/racecar/install/my_racecar/share/my_racecar -xtype l\nfind /home/racecar/install/my_racecar/share/my_racecar -xtype l -delete\nls -l /home/racecar/src/my_racecar/mapping/launch/distributed_mapping_vehicle.launch.py")],
    }),
    ("issue", "第二圈起里程计严重漂移、高速外围漂移（2026-10-11，未解决）", {
        "现象": "第一圈很稳，第二圈起地图重影；2.5 m/s 以上外围漂移，近处锥桶准。",
        "排查": ["推测（未证实）：第二圈回环没触发，或扫描匹配被漂移的里程计朝向带偏；高速时雷达 10 Hz 一圈内车走约 25 cm，产生运动畸变，远处点偏得多。",
                 "mapping_status_logger 新增每 5 秒记录 odom、slam、corr 三组 [x, y, yaw°]（corr = map→odom_pc）。判读：corr 偏航缓慢单调增大 = 里程计在漂、SLAM 在拉；corr 一直近 0 但图乱 = 匹配问题；corr 突变 = 回环生效或误闭合。再用 -NoOdom 对比，纯激光能建好说明问题在里程计朝向。"],
        "解决": ["已改（未提交、未实车测）：mapper.yaml / mapper_native.yaml 加 distance_variance_penalty 1.0、angle_variance_penalty 1.8、minimum_distance_penalty 0.3、minimum_angle_penalty 0.6；loop_search_maximum_distance 4→6；loop_match_minimum_response_coarse/fine 0.35/0.45→0.30/0.40。",
                 "待选方案：①建图圈慢跑（约 0.5 m/s），高速只跑路径；②录 /scan 和 /encoder_imu_odom 的 rosbag 离线建图调参；③雷达提频、打开 scan_tf_bridge 的 deskew（要配对 stamp_at_end）、查时间戳偏移；④用赛道已知几何（锥桶 / 墙）配准还原真实跑道；⑤标定轮径系数、IMU 零偏。"],
    }),
    ("issue", "10-17 20:08 那趟：位姿一直是 (0,0)，图只有原地一圈扫描", {
        "现象": "run_20261017_2008_filtered：5 分钟 3049 帧扫描、15212 条里程计都收到了，但 odom / slam 一直是 (0, 0)，日志最后报 stalled，map.png 只有放射状一团。",
        "原因": "车没动，或里程计在发消息但位置没更新。待确认。",
        "解决": [C("# 小车：推车时看 x、y 变不变\nsource /opt/ros/humble/setup.bash\nsource /home/racecar/install/local_setup.bash\nexport ROS_DOMAIN_ID=42\ntimeout 5 ros2 topic echo /encoder_imu_odom --field pose.pose.position")],
    }),
    ("issue", "保存的地图打不开（只看到 .raw / .pgm）", {
        "原因": "map.pgm 是 ROS 标准地图，Windows 看图软件打不开；map.raw 是原始栅格数据，不是图片。",
        "解决": "save_mapping 增加输出 map.png（标准库 zlib 写灰度 PNG），双击即可看。旧地图可用同样方法补转。",
        "结果": "run_20261017_2008_filtered 已补出 map.png。",
    }),
    ("issue", "车快时锥桶黑点淡、模糊（2026-10-19，新增实体化障碍层）", {
        "现象": "高速建图时锥桶在 map.png 里只剩零散一两个黑点，有的缺；墙有断口。",
        "排查": ["统计已保存地图的像素：只有 0（障碍）、205（未知）、254（空闲）三种值，没有中间灰度。所以“淡”不是颜色浅，是障碍点又少又散。",
                 "slam_toolbox 每格按“命中 / 穿过”比例判障碍：车快时锥桶只被扫到一两次，旁边擦过的射线多，被判成空闲或未知。"],
        "原因": "slam_toolbox 的占据栅格本身的特性，不是参数问题；只调阈值会把走动的人一起留下。",
        "解决": ["新增 mapping/scripts/obstacle_layer（快捷键 4 开、2 不开），发 /map_solid（只用来看）。第二版做法（第一版的问题见下一张卡片）：",
                 "1) 位置只用 SLAM 原图 /map：找“小而孤立”的障碍点团（相邻 1 格内连成一团，大小 <= 0.35 m），换成实心圆；墙和其他障碍保持原样。",
                 "2) 孤立：外环（离锥桶边缘 0.15~0.40 m）别的障碍格 <= 10（墙碎片远多于此），且外环里已探明空闲格 >= 60%（房间外、射线边缘的噪点周围大多是未知）。",
                 "3) 半径：近处（2.5 m 内）单帧里的短段做圆拟合（最小二乘 + Gauss-Newton）取中位数；单帧只用相对形状，和位姿无关。会移动的物体（人腿）在里程计坐标系里短时跟踪（3 s）后排除。",
                 "4) 画图：先清掉锥桶周围 0.25 m 内原来的散点，再画和圆有重叠的格子（8 cm 半径 = 3x3 格）。",
                 "人：filtered 模式的原图本来就基本不留走动的人，实体层只把原图已有的点团变实。原图里一个点都没有的锥桶补不出来。",
                 "save_mapping 有 /map_solid 时多存 map_solid.* 和 solid.json；run_slam_windows.ps1 加 -NoSolid。"],
        "结果": ["实车 10-19 20:17 那张原图离线加工：画出 77 个锥桶，和原图锥桶排布一致；房间外噪点全部去掉；拟合半径 0.082 m（1581 个单帧样本）。",
                 "房间里还有几个零星的圆，原图那里本来就有小黑点（可能是椅子腿等真实小物体）。这些和靠边的点在第三版里处理，见下一张卡片。"],
    }),
    ("issue", "实体层第二版：靠边的东西也被画成锥桶；刚开始建图出现特别大的圆（2026-10-19）", {
        "现象": "赛道锥桶正确，但门口一簇杂点、靠墙的几个点也被画成了圆；建图刚开始时有几个圆明显比锥桶大。",
        "排查": ["用 20:27 那趟保存的原图离线加工，彩色差异图对比（红 = 新增，蓝 = 清掉）。",
                 "到房间外未知、到别的点团的距离：门口杂点 0.22~0.34 m 内就有别的小点；赛道锥桶最近邻居 >= 0.5 m（内圈间距只有 0.5~0.6 m）。",
                 "外环 0.25~0.49 m 时内圈锥桶被邻居误拒；只看“任何未知”时锥桶背后的阴影把旁边锥桶也判成边缘；只看长墙时门口的断墙不算。"],
        "原因": "边缘的定义太粗；半径样本里混进了墙角、平墙段拟合出的大圆。",
        "解决": ["边缘 = 和地图边界连通的未知（房间外；被空地包住的阴影不算）+ 长度 > 3 m 的墙 + 贴着房间外边 1 m 内、>= 0.4 m 的物体（断墙、门框）。离边缘 edge_margin（1.0 m）内的点团不加工。",
                 "相距 1 个空格内的碎块先合成一团（淡锥桶常被拆成两三块）；外环改为锥桶边缘外 0.08~0.25 m，只数别的团的格子，> 2 格不加工。",
                 "半径样本只收“凸向雷达”（圆心在点后面）且覆盖 >= 60° 的圆弧。"],
        "结果": ["20:27 原图：赛道锥桶基本都画了；底边、顶边靠墙的和门口大部分不再加工；门口还剩 2 个圆（原图里也是圆点，可能真是锥桶）。顶边 1 个赛道锥桶离墙凸起不到 1 m，没被加工。",
                 "edge_margin 调到 1.2 / 1.4 会误伤更多顶边锥桶，所以保持 1.0。模拟走廊只有 4 m 宽，锥桶离墙 0.8 m，默认参数下一个都不加工（edge_margin 0.5 时 12/12）。",
                 "实车 10-19 20:54、21:09 两趟带实体层建图：实体层记录 68 / 70 个锥桶，半径 0.0815 / 0.0818 m（各约 350 个单帧样本）。两趟都没按 3 保存，用 autosave 原图离线复查："
                 "赛道锥桶都是大小一致的圆，墙和房间外按原图，没有假黑块，也没有过大的圆；个别离内部短墙近或点团太碎的锥桶没加工。",
                 "还没验证：按 3 保存时 map_solid.png / solid.json 是否正常写出（autosave 只存原图）。离线复查命令见八、调试命令 8.5.1。"],
    }),
    ("issue", "实体层第一版：实车出现大片不存在的黑块和斜线（2026-10-19）", {
        "现象": "第一版实体层在实车上锥桶变清晰了，但空地上出现很多黑色条带、大黑块，整张图像被斜着叠了好几层。",
        "排查": ["同一时刻自动保存的 SLAM 原图是干净的（锥桶、墙都对），黑块全是实体层画的。",
                 ".run1-live.jsonl 最后一行 corr（SLAM 对里程计的修正）偏航 21.75°：里程计朝向漂得很厉害，SLAM 一直在往回拉。"],
        "原因": "第一版自己累计点：用“当时”的位姿把雷达点画到地图上，之后不再改。SLAM 修正位姿（含回环）时会重新摆放所有历史扫描，原图始终对；实体层不会跟着改，同一面墙在不同时刻被画在不同角度。模拟里里程计不漂，所以没暴露。",
        "解决": "改成只在 SLAM 原图上加工（见上一张卡片），不自己累计任何点的位置。",
    }),
    ("issue", "PowerShell 里敲 2 不执行命令", {
        "原因": "PowerShell 把 2 解析成数字字面量，不能做别名或函数。",
        "解决": "在 PowerShell 配置文件里用 PSReadLine 的回车键处理：整行只有 2 / 3 时替换成建图 / 保存命令再执行。",
    }),
    ("issue", "VS Code 终端里敲 4 只回显数字 4（2026-10-19）", {
        "现象": "在 VS Code 的终端敲 4 回车，只输出一个 4，没有开始建图。",
        "排查": ["电脑没装 pwsh，VS Code 用的是 Windows PowerShell 5.1；新开的 powershell 窗口加载配置文件正常，回车键处理在。"],
        "原因": "推测那个终端是在配置文件里加 4 之前开的，没加载新的回车键处理。",
        "解决": ["已开的终端先运行 . $PROFILE，或者关掉重新开一个终端再敲 4。",
                 "还不行时运行下面三条，看回车键是不是被 VS Code 的 shell integration 接管了，或者用的是不是另一个配置文件：",
                 C("# 电脑 PowerShell（出问题的那个终端）\nGet-PSReadLineKeyHandler -Key Enter\n$PROFILE\nTest-Path $PROFILE")],
        "结果": "之后 21:09 那趟已带实体层建图成功（是否用的快捷键 4 未确认）。",
    }),
    ("issue", "现场 WiFi 丢包：断几秒地图就对不上（2026-10-21）", {
        "现象": "比赛现场网络差，局域网还连着但经常丢包。原来的方式下，丢掉的扫描就没了；断几秒、车又快时，两帧之间车走得太远，SLAM 匹配不上，地图错位或重影。",
        "原因": ["电脑直接收 /scan 用的是 Best Effort：丢了不重传。一帧扫描约 8 KB，要拆成好几个 UDP 包，丢一个整帧作废。",
                 "电脑用的是 async_slam_toolbox：忙不过来时跳帧，网络恢复后就算一次来很多帧也不会全处理。"],
        "解决": ["小车新增 scan_relay：在车上本地收 /scan、/encoder_imu_odom（不经 WiFi，不丢），以 RELIABLE + TRANSIENT_LOCAL + 留存 60 s 重发为 /scan_relay、/odom_relay（去掉 intensities，体积减半）。丢包自动重传；断线后（哪怕 DDS 已判定掉线、重新发现）把最近 60 s 补发过来。",
                 "电脑 scan_tf_bridge：收到的扫描进排队区，按时间戳排序、去重（补发会重复），等该时刻的里程计到了再处理；里程计缓存从 2 s 加到 120 s；补发时每秒最多喂 SLAM 约 80 帧。刚连上时丢掉车上留存的旧数据，只从最近 2 s 开始。",
                 "SLAM 默认换成 sync_slam_toolbox_node（逐帧处理，不跳帧）；-Async 可换回。",
                 "source:=auto：车上有 scan_relay 就用，6 s 内没有而 /scan 在发就退回直接收并报红字。",
                 "兜底：小车敲 1 时同时 ros2 bag record /scan /encoder_imu_odom；网络断得比 60 s 还久、实时图坏了时，拷回电脑用 -Bag 离线重建（没正常结束的包自动 reindex）。",
                 "/scan_link 每 5 s 发网络统计，状态日志记成 link 字段，[监控] 多一行“网络”。"],
        "结果": ["无车模拟（fake_car3，1 m/s 跑 30 s，挂起电脑端 scan_tf_bridge 模拟断网）：断 12 s，relay 收全 299/300 帧（118 帧晚到、最大延迟 12.1 s、补不回 0），直接收方式丢 121 帧；断 25 s（超过 DDS 默认 20 s 租期），relay 仍收全 379/380 帧（220 帧补发、最大延迟 22.3 s）。补发进来的图和不断网的一样干净，没有重影。",
                 "离线重建：录 30 s 的包（强杀录包，没有 metadata.yaml），-Bag 自动 reindex，2 倍速放完 299/299 帧，自动保存出 map.png、map_solid.png。",
                 "车上没有 scan_relay 时自动退回直接收 /scan，299 帧正常建图。",
                 "10-22 上车：改用 git 同步（电脑提交 aeb9085 → git push car main:from-pc → 小车 merge --ff-only + colcon build，scan_relay 装成指向源码的链接，ros2 bag 可用），不用第 4 章的 scp。",
                 "10-22 19:37 实车（车停着没动）：source=relay，整趟汇总（8.5 的命令）收到 2610 帧、补发晚到 80 帧、补不回 0、最大延迟 2.63 s（车停着也有短暂丢包，被补回来了）；车上录到包 /home/racecar/bags/map_20270102_032655（车上时钟错，名字是 2027），已拷到 D:\\ubuntu\\slam\\bags\\。",
                 "11-01 操场实车（车动了，cmd_vel 有几千条）：09:54 那趟 收到 1721、补不回 5、最大延迟 0.26 s；10:01 那趟 收到 3404、晚到 441、补不回 16、最大延迟 20.01 s（中间断网很久）。lost 不为 0 → 验收未通过。数据：D:\\ubuntu\\slam\\maps\\run_20261101_0954_filtered、run_20261101_1001_filtered 的 .run1-live.jsonl。",
                 "同两趟的 SLAM 位姿被地面假墙带偏（见下一张卡片），地图本身不可用；要等地面线解决后重新验收，并查补不回的 16 帧是不是超过了车上 60 s 留存。",
                 "11-04 用户要求全删：scan_relay、排队补帧、去重、sync SLAM、车上录包、-Bag 离线重建都已删除，电脑直接收 /scan、SLAM 回到 async（手册 3.6 节改成高速建图）。用户说跑车时离路由器只有约 2 m，断流很少；11-03 各趟车动时扫描也基本稳定 10 帧/s。地图漂移的主因是两个里程计和车速（见“两个里程计同时在发”“高速 3 m/s 建图”两张卡片），不是 WiFi。"],
    }),
    ("issue", "车停着时地图边上出现不存在的“墙”，实体层一个锥桶也没画（2026-10-22）", {
        "现象": "快捷键 4 建图，车停着。rviz 里视野外沿有几条长直线（红点 + 黑格），用户说那里什么都没有；Map Solid 层和原图一样，锥桶没有画成实心圆。",
        "排查": ["本趟 .run1-solid.json：cones 0、not_isolated 45。用 autosave 原图离线重跑 solidify：45 个小点团全部因“靠边”被拒。",
                 "车上录的包（2391 帧）逐角度取中位数：驱动报 range_max 200 m，但实际最远点 9.04 m，无回波给 inf（15%），没有固定在某个距离上的假点；反射强度全为 0。",
                 "那几条直线（车左约 4.5~5.5 m、车后约 2 m、车前约 4 m）每帧都有，帧间距离抖动中位 5~8 mm，和 1 m 内的近处物体一样稳。",
                 C("# 电脑 Git Bash：逐角度统计包里的 /scan（脚本在会话临时目录，需要时重写：sqlite 读 db3，手动解 CDR）\npython bagscan.py D:/ubuntu/slam/bags/map_20270102_032655/map_20270102_032655_0.db3")],
        "原因": ["地图：车没走够 0.2 m / 0.15 rad，slam_toolbox 不往图里加新扫描，图里只有第一帧；min_pass_through 3 要求格子被 3 条激光穿过才算空地，约 3 m 外激光太稀，所以远处是零散的点、外沿一圈灰色。车一动，多位姿取多数会把它补齐。",
                 "实体层：把“和地图外圈连着的未知区域”当成房间外，离它 1 m 内的点团都按墙边不加工。开阔场地里这一圈只是看不远，不是墙，所以全被拒。只删这条规则会把远处墙的零散点画成 29 个假锥桶。",
                 "那几条直线：不是量程边缘（离 9 m 还远）、不是闪烁噪点（每帧都在）。用户确认那里没有东西 → 最可能是雷达安装倾斜，扫描面扫到了地面（待量倾角确认）。"],
        "解决": ["待做（还没改代码）：① 量雷达倾角（车停平地，看扫描面在哪个方向、多远碰到地面），确认后在安装上调平，或在桥里按倾角滤掉地面点；",
                 "② 实体层“边缘”只算连成长条的真墙，视野外沿不算；③ 锥桶判断改用单帧扫描证据（一小段、两边邻点更远或无回波），按不同位姿取多数，原地重复扫描只算一个位姿。用包 -Bag 回放验证后再上车。",
                 "11-01 定下两条路（用户还没选）：① 调平雷达，前端垫高到水平或略上仰 0.5~1°（俯角 < 1° 时地面线推到 8 m 外）；② scan_tf_bridge 里加地面线过滤：操场静止录包标定地面线在雷达系的方向和距离，每帧在标定值附近找长直线，删线上和线外的点（线外本来不可能有回波），只改电脑端，-Bag 回放验证后用。"],
        "结果": ["11-01 操场（空地、只有人）确认是雷达扫到地面：车前方约 3 m 一条长直线，跟车走；按雷达离地约 0.15 m 估前倾约 3°（离地高度未量）。",
                 "影响比原来想的大：车动起来 SLAM 也被带偏——10:01 那趟里程计 (2.72, 3.34, -149°)、SLAM (0.42, -0.22, 148°)，修正量 (-3.8 m, 0.6 m, -62°)；09:54 那趟 SLAM 认为车跑出约 30 m。空地里唯一像墙的就是这条跟着车走的线，SLAM 拿它匹配。“运动时多帧取多数能洗掉”在空地不成立。",
                 "同日另一张图里车前没出现直线（用户是否调过雷达未确认）。“只建一半”不是 bug：没回波（inf）的方向 SLAM 不画，空场那一侧保持未知。",
                 "IMU 不用调平：imu_encoder_mix 只取 IMU 融合姿态的 yaw 减开机 yaw，加速度不用；下游都只取 yaw。要注意的是固定牢、开机时摆正停稳、磁场干扰。",
                 "11-03 做了路线 ②（不用标定、不用录包）：scan_tf_bridge 加 GroundLineFilter，自己认地面线——雷达系里一直不动、而车沿它的法向走了 1 m 以上（或转了 0.5 rad）的长直线。认出后每帧删掉线上和线外的点，填 NaN（karto 源码 Karto.h 里 NaN 既不画障碍也不清空射线；inf 会被当成一直没东西）。默认开（-Ground auto），-Ground off 关，-Ground fixed -GroundRho -GroundDeg 直接给定（静止建图用）。",
                 "无车模拟（10 Hz、720 点，地面线 3 m、粗糙 3 cm，1.3 m 锥桶方阵）：1.5 m/s 第 9 帧认出、0.3 m/s 第 34 帧认出，法向 / 距离误差 < 0.1° / 0.01 m，认出后锥桶点保留 99%、地面点残留个位数；走廊（雷达水平、朝前墙开）、静止都不误认；每帧约 2 ms。脚本：临时修改\\windows建图linux建图分布式通信\\windows\\模拟测试\\地面线\\test_ground.py（命令见八、8.6.2）。",
                 "还没上车：操场跑一趟，看建图窗口 [监控] 地面线一行（手册 3.4.3）和 rviz 车前那条线有没有消失。雷达调平仍建议做（地面线推远，车前能看得更远）。",
                 "11-05：实车上把挡板认成了地面线，判据已改，见下一张卡片。"],
    }),
    ("issue", "地面线过滤把比赛挡板也删了（2026-11-05）", {
        "现象": "用户：开了地面剔除后，比赛设置的挡板也被剔除了。rviz 里当前扫描（红点）有一长段弯挡板，地图（黑格）里那一段只剩零散几个点。",
        "排查": ["翻 11-03 以来全部建图的 .run1-live.jsonl（/scan_link 里的 ground 字段）：认出过地面线的 4 趟，法向 / 距离是 -44.4° / 3.26 m（11-03 20:57）、128.8° / 1.9 m（21:39）、-46.9° / 2.99 m（22:07）、-27.9° / 4.65 m（11-05 17:00）。雷达往前低头，真地面线法向应接近 0°（正前方）；这些在侧面、身后，是挡板。",
                 "旧判据：雷达系里不动 + 车沿法向“累计”走 1 m 或“累计”转 0.5 rad，累计的是每帧的绝对值。车沿弯道挡板转弯时挡板跟着车转、在雷达系里不动，转角很快累计够；贴着直挡板时左右晃动的绝对值也会一直累加。",
                 "无车模拟（test_ground.py，脚本在会话临时目录）：旧判据下“车沿半径 5 m 的弯、挡板在外侧 0.8 m”第 9 帧就认出，法向 -66.6°；直挡板晃动、朝挡板开再拐弯不认；真地面线第 11 帧认出。"],
        "原因": "地面线判据只看“雷达系里不动 + 车动了”，沿弯道挡板行驶时挡板也满足；认出后删“线上和线外”的点，等于把那一侧整片（连挡板）删掉。",
        "解决": ["scan_tf_bridge 的 GroundLineFilter 换判据：候选线每次被看到，把它在里程计系里的线段存下来；车后来跑到某次存下的线段另一侧（越过 0.10 m、越过点在线段两端 0.2 m 以内），同时这条线在雷达系里还在，才认定。地面线跟着车走，车开得过去；挡板车开不过去。参数 ground_learn_move / ground_learn_turn 删掉，新增 ground_cross_margin（默认 0.10 m）。",
                 "同一模拟新判据：直挡板晃动、弯道挡板、朝挡板开再拐弯都不认；真地面线（车前 2 m、车速 0.1 m/帧）第 21 帧认出（要开过 2 m，比旧判据晚，正常）。",
                 "加两种投票地图（vote_map，手册 3.4.5）：只投票 / 投票 + 地面剔除，rviz 实时看、保存时都存、报告并排显示并数出差了多少障碍格，以后地面剔除删错了一眼能看出来。"],
        "结果": ["车上包离线回放（replay.ps1）：110741（= 11-03 22:07，旧判据当时认出 -46.9°）、101953（= 20:57，旧判据认出 -44.4°）两趟新判据都没认出地面线；两种投票图都存出来、报告并排，锥桶位置和 SLAM 原图一致（D:\\ubuntu\\slam\\maps_replay\\1105_*_vote*\\）。",
                 "vote_map 第一版每帧投票 50 ms，回放时饿着 ROS 回调，丢了 228 帧、回环后重投晚了约 50 s；改成查表后 12 ms/帧，重放 110741：2001 帧全收、0 丢，按优化结果重投 188 次（位姿图节点 237 个、被优化挪动的 116 个）。",
                 "待确认：同一个包第二次回放 SLAM 有一次 corr 突跳约 2 m / 11°，第一次没有。async SLAM 每次结果本来就不完全一样，也可能是 vote_map 抢 CPU；实车建图如果突跳变多，先加 -NoVote 对比。",
                 "还没在实车上跑：建图时看 [监控] 地面线一行，法向应接近 0° 或一直“还没认出”；保存后报告“两种投票地图”右图不应少挡板。"],
    }),
    ("issue", "建图突跳：里程计经 WiFi 晚到，转弯时 SLAM 来回摆（2026-11-05）", {
        "现象": "11-05 17:57 那趟（run_20261105_1757_fast，车约 1 m/s 跑两圈）报告：SLAM 突跳最大 1.61 m / 14°，超过 1 m 或 10° 的 7 次（09:59:08、09:59:13、09:59:18、09:59:23、09:59:33、09:59:53 …，车上时钟）。用户要求查原因。",
        "排查": ["状态日志的 corr（SLAM 对里程计的修正）：转弯时冲到 10~12°（09:59:03 12.4°、09:59:13 10.2°、09:59:23 10.6°、09:59:53 11.1°），回到直道又落回 0° 左右。不是一次跳过去不回来，是“转弯就歪、直道拉回”，像时间对不上，不像匹配错。",
                 "建图桥日志（C:\\Users\\lzc2\\.ros\\log\\python_<pid>_*.log，第一行是 scan_tf_bridge 的那个）：每 5 s 转发 30~50 帧，其中“插值失败”20~40 次；“扫描-最新里程计时间差”+0.3~+1.8 s。即扫描到电脑时，同一时刻的里程计还没到。",
                 "同一套代码离线回放车上录的包（车上录包没有 WiFi）：插值失败几乎为 0（110741 全程 24 次，都在开头）。所以是实车链路的问题。",
                 "扫描订阅是 best-effort（丢了就丢），里程计是 reliable（丢了要重传）：WiFi 丢包时里程计排队重传、成批晚到，扫描照常到。状态日志里 odom_rate 在 10~54 条/s 之间跳也说明里程计是一阵一阵来的。"],
        "原因": "建图桥扫描一到就处理。里程计晚到超过 0.3 s 时插值失败，只能用最新一条（已经是 0.3~1.8 s 前的）位姿、也不去畸变；车转弯约 45°/s，0.3 s 就差十几度。SLAM 拿扫描匹配把它纠回来（corr 跟着歪），到直道里程计和扫描又对上，corr 再拉回去，报告就记成一次次“突跳”，地图在弯道容易重影。",
        "解决": ["scan_tf_bridge 加 odom_wait（默认 1.0 s，0 = 旧做法）：扫描先进队列，等里程计时间戳追上扫描时间戳再处理，最多等 1 s；按顺序处理，SLAM 看到的扫描晚不到 1 s，rviz 里感觉不出来。",
                 "[监控] 和 /scan_link 加“扣住等里程计 N 帧（最长 X s，等满仍没等到 M 帧）”；报告结论表加“里程计晚到（WiFi）”一行（手册 3.4.2）。",
                 "离线验证工具：player.py --odom-lag D（每 2 s 一段 0.3~D 秒里程计卡住、之后一起发，固定随机种子），replay.ps1 -OdomLag D -OdomWait W。"],
        "结果": ["车上包 110741 回放 + player.py --odom-lag 1.2（每 2 s 卡 0.3~1.2 s，固定种子）：改前（-OdomWait 0）2000 帧里插值失败 501 帧（25%，和实车同量级）；改后（-OdomWait 1.0）2001 帧里失败 24 帧（都在开头没里程计时），扣住等里程计 1502 帧、最长 1.00 s、等满没等到 35 帧。桥这一层修好了。",
                 "但 SLAM 结果看不出差别：改前 / 改后 corr 5 s 突跳 3 / 4 次，第一圈左头 U 弯 corr 都偏到约 -15°，回环时拉回（改后那次 17.5° 就是回环拉正）。不模拟晚到的两次回放（vote / vote2）同一个弯也到 -4° / -11°，突跳 0 / 3 次：async SLAM 每次结果不同、这个弯本身难，噪声盖过了差别。回放的突跳次数不适合当判据。",
                 "实车 17:57 的样子是“转弯 corr +10~12°、直道回 0”，和回放那种“偏了不回来”不一样，符合“用了旧位姿”；实车每 5 s 失败 20~40 帧（约 70%），比模拟重得多。结论：原因有日志证据，修复在桥这一层验证过，对实车突跳的效果还要实车跑一趟确认（看报告“里程计晚到”一行和突跳次数）。",
                 "顺带：17:57 那趟地面线没认出（ground null），两种投票图完全相同（sha 一样），vote_map TF 查不到 11 帧、重投 141 次——交接里要核对的这几项都正常。"],
    }),
    ("issue", "操场走长线地图偏移、同一批锥桶画成两份（2026-11-03）", {
        "现象": "操场（锥桶约 1.3 m 等间距，空地没有墙）建图，直线开一段地图就偏；rviz 里当前扫描的红点和地图黑点错开小半个到一个锥桶；同一批锥桶被画成左右两份；空地里有长黑弧线、斜黑线（用户确认那里什么都没有）。",
        "排查": ["20:02 那趟 .run1-live.jsonl（命令见八、8.5）：车 12:06:26 开动，5 s 后 SLAM 与里程计的修正量 corr 就到 (0.15, 6.66 m, -67°)——不是慢慢漂，是几帧之内朝向错了几十度。之后网络断过（补不回合计 64 帧、晚到 618 帧），里程计 18 m 一跳是补发追上来，不是里程计坏。",
                 "参数：10-11 为“第二圈朝向漂”把偏离里程计的惩罚调小（1.0 / 1.8 / 0.3 / 0.6），搜索范围 ±0.5 m / ±28.6°，回环阈值放宽到 0.30 / 0.40。扫描匹配以里程计预测为中心搜索，搜索范围只需盖住里程计误差，旧注释“车快所以要大”不对。",
                 "长黑弧线、斜黑线 = 跟着车走的地面线在车转弯、位姿错时被一段段画进图（见上一张卡片）。"],
        "原因": "空地里激光约束弱（锥桶稀疏、排列重复，再加一条跟车走的地面线），参数又让 SLAM 少信里程计、在大范围里找，匹配对错锥桶 / 被地面线拉着走；回环阈值低还可能接错。",
        "解决": ["两个 mapper yaml：搜索改回默认 ±0.25 m / ±20°（小于半个锥桶间距）；惩罚改回默认 0.5 / 1.0 / 0.5 / 0.9；回环阈值 0.35 / 0.45、搜索 4 m。",
                 "地面线过滤（上一张卡片）。"],
        "结果": ["11-03 20:57 那趟（新参数）：两圈 3.5 分钟、62 m，corr 5 s 内最大 0.53 m / 3°，结束 1.44 m / -2.7°，锥桶一份、两圈对得上（报告 run_20261103_2057_filtered\\run1\\report.html）。比 20:02 那趟（67°）好得多。",
                 "21:02 那趟：车开约 2 m/s、只跑了约 15 s，corr 两次突跳 2.1 m / 25°、3.3 m / 18°，锥桶又散了；同时里程计有两个（见下一张卡片），地面线还没认出。",
                 "11-04 汇总 11-01、11-03 全部记录：漂得最厉害的几趟（11-01 10:15 里程计 5 s 跳 58 m、11-03 21:23 corr 跳 12 m）都是两个里程计同时在发；只有一个里程计的 21:58 那趟车速约 1.9 m/s，corr 也跳了 12 m；车速 0.6 m/s 的 20:57、约 1 m/s 的 22:07 两趟 corr 都在 0.7 m 内。之前说的“主因是 WiFi 断流”不对，已更正。"],
    }),
    ("issue", "两个里程计同时在发：车一跑路径，地图就漂（2026-11-03，11-04 修）", {
        "现象": "网络不好、车很快时建图还是漂；用户怀疑“里程计出现两个”（用户 11-04 确认：车上同时有两个里程计在发，一个融合 IMU + 雷达，一个融合 IMU + 轮速）。",
        "排查": ["状态日志里里程计条数 / 秒（报告“里程计是不是只有一个”那一行自动算）：车停着、只敲了 1 时约 50 条/s；车一开动（跑路径）就变成约 85~100 条/s，车停下后也不回落。20:13、20:57、21:02 三趟都这样。",
                 "11-04 核对 11-01、11-03 全部记录：只有一个发布者的 21:33、21:39、21:58、22:07 四趟，车动时 40~49 条/s——一个里程计车停、车动都约 50 条/s。11-03 会话里一度改成“车动约 100 条/s 是正常的”，那是错的，已改回。11-01 各趟车动时 73~93 条/s；10:15 那趟里程计从 50 涨到 93 条/s 的同一个 5 s，位置从 (2.2, 0.8) 跳到 (33.5, −48.4)，之后 corr 乱到几十米。",
                 "21:23 那趟（run_20261103_2123_filtered）直接抓到：13:24:01 发布者 1 个；13:24:06 cmd_vel 从 0 涨到 339（开始跑路径）同时变成 2 个，这 5 s 里程计约 110 条/s；odom 位姿 5 s 内从 (2.4, 3.8) 跳到 (−9.4, 3.0)、角度来回跳一百多度。corr 4 次突跳（2.9 m / 32°、2.3 m / 16°、2.8 m / 29°、12.2 m / 77°）都在两个发布者期间或紧接其后；12.2 m 那次车已停（cmd_vel 不再增加、odom 不变），是前面叠出重影后 SLAM 匹配错位。",
                 "launch/move_test.launch.py 的 start_odom 默认 true：跑路径时自己再起一个 imu_encoder_mix；快捷键 1 已经起了一个 → 两个同名节点同时发 /encoder_imu_odom。"],
        "原因": "两个里程计各自以自己启动那一刻为原点、各自积分，两份位姿交替发在同一个话题上。scan_tf_bridge 收到的位姿来回跳，SLAM 被来回拉；两份轨迹差得越多（一个 IMU + 雷达、一个 IMU + 轮速）漂得越厉害。车上的路径跟踪（get_local_path / pure_track_test）也收到来回跳的里程计——正是“室外跑路径车不动”的嫌疑之一（五、AB 路径那张卡片）。",
        "解决": ["11-04（用户要求：里程计只留快捷键 1 那一个，话题改成有识别度的名字 /key1_odom）：快捷键 1 的 imu_encoder_mix 输出 remap 成 /key1_odom；move_test、move_new、final_start、save_path、odom_move_test 五个 launch 和 record_seg.py 都改订 /key1_odom，跑路径 / 录路径默认不再自己起里程计（start_odom 默认 false）；建图桥只收 /key1_odom。别处再发 /encoder_imu_odom 的都没人收。C++ 没改（靠 launch remap）。",
                 "11-04 下午改方案（用户：跑路径那个里程计他们有用，恢复原样，改我们自己的话题名就行）：5 个运动 / 录路径 launch 和 record_seg.py 恢复成 HEAD 原样（照旧自己起 imu_encoder_mix 发 /encoder_imu_odom）；只改快捷键 1：节点名 key1_imu_encoder_mix、话题 /key1_odom，建图桥只收它。两份里程计在不同话题上，建图不再被跑路径那份拉；节点名也不同，不会重名。改动备份：D:\\ubuntu\\slam\\bak_20261104_跑路径key1改动.patch。",
                 "建图窗口两种红字：“车上有 N 个 /key1_odom 发布者”（发现计数）、“/key1_odom N 条/s（一个里程计约 50）”（车动时超过 70 条/s，发现计数经 WiFi 偶尔数不全，条数更可靠）。建图报告“里程计是不是只有一个”两样都看。",
                 "查是谁在发（小车，建图或跑路径时）：",
                 C("# 小车\nsource /opt/ros/humble/setup.bash\nexport ROS_DOMAIN_ID=42\nros2 topic info /key1_odom -v | grep -E \"Node name|Publisher count\"\nros2 topic info /encoder_imu_odom -v | grep -E \"Node name|Publisher count\"\nps -ef | grep -E \"imu|odom\" | grep -v grep")],
        "结果": "电脑仓库改完、假数据冒烟通过；还没上车（要按手册第 4 章 git 同步 + colcon build）。上车后边建图边跑路径：/key1_odom Publisher count 1（key1_imu_encoder_mix）、/encoder_imu_odom Publisher count 1（imu_encoder_mix，跑路径的），报告里车动时约 50 条/s、没有红字。",
    }),
    ("issue", "高速 3 m/s 建图：一圈扫描内车走了 30 cm，两圈对不上（2026-11-04）", {
        "现象": "比赛要求高速（约 3 m/s）建图。11-03 21:58 那趟只有一个里程计，车速约 1.9 m/s，corr 仍跳 12 m；车速 0.6、1 m/s 的两趟 corr 在 0.7 m 内。用户跑两圈，两圈的轨迹有偏移。",
        "排查": ["雷达转一圈 0.1 s，3 m/s 时一圈里车前进 30 cm、转弯时还转一个角度；整圈点当成同一时刻拿去匹配，墙和锥桶都变形。scan_tf_bridge 早就有逐点去畸变，但默认关：要知道扫描时间戳是一圈的开始还是结束，设反了比不去畸变还差。",
                 "车上录包（10-22 的 map_20270102_032655）：/scan 时间戳比录包时刻只早中位 0.5 ms、/encoder_imu_odom 0.3 ms → 驱动转完一圈才打时间戳，是一圈的结束。",
                 "假数据（10 × 8 m 场地，v 3 m/s、w 0.8 rad/s）：不去畸变 点误差中位 7.9 cm、95% 32.7 cm；去畸变 + 一圈结束 0.3 cm / 1.7 cm；去畸变但设反 21.8 cm / 61.3 cm。"],
        "解决": ["去畸变默认开：scan_tf_bridge、两个电脑端 launch 的 deskew、stamp_at_end 默认都改 true（-NoDeskew 关）。",
                 "两圈的约束：新增 mapper_fast.yaml（-Fast），回环搜索距离 4 m → 8 m，第二圈回到第一圈的地方时更远的偏差也能拉回去；接受阈值、扫描匹配搜索范围、偏离里程计惩罚不动（11-03 放宽它们会把锥桶对错，见“操场走长线地图偏移”）。",
                 "里程计权重：-Odom ekf（下一张卡片），EKF 里轮速的不确定度随车速变大（0.05 + 0.05×|v| m/s，打滑），航向主要信 IMU 角速度。"],
        "结果": "代码完成、假数据通过；-Fast 和 EKF 的效果要用实车包对比：用户拷车上 /home/racecar/bags/ 的包后，同一份数据分别用 mapper.yaml / mapper_fast.yaml、开 / 关去畸变各重建一次，看报告 corr 和锥桶重影。",
    }),
    ("issue", "赛道拟合：地图形状和规则赛道对不上 / 认错赛道（2026-11-04 新增 fit_track）", {
        "现象": "用户要求：比赛赛道是规则里 A/B/C/D 四种随机一种，建图后处理时在描黑好的地图上和这四种拟合、校正，尽量和规则图一样。",
        "排查": ["模板：外框 16.8 × 5.5 m，两头外圈半径 2.75 m、内圈 1.25 m，车道 1.5 m，锥桶约 1.25 m 一个。A/B：上边左半向下凹的弯、右半 135° 折线区（只有地面黄线，雷达看不到）；C/D：上边左半折线区、下边右半向上凸的弯。A/C 左头内圈直角，B/D 圆弧。弯的锥桶位置从规则截图量，误差约 0.2 m。",
                 "第一版刚性拟合一开始就放开比例，外圈锥桶被吸到内圈线上，比例算成 0.946（真值 1.03）、A 认对但分数 0.50；改成前 25 次固定比例 1、再放开细调，初始朝向每 20° 试一次，分数里乱点只取最近 80% → 0.16。"],
        "原因": "（新功能）",
        "解决": ["新增 mapping/scripts/fit_track：四种模板各自做相似变换拟合（比例 0.9~1.1），分数 = 锥桶到边线距离（最近 80%，截断 0.8 m）+ 0.5 × 模板锥桶到地图锥桶距离（截断 1 m）；选分数最小的，-Track X（环境变量 FIT_TRACK）可指定。",
                 "11-04 用户追加要求：和赛道有差异的地方、建图偏移都强制校正，校正成和 4 种赛道的锥桶形式差不多；黄线折线区可能有锥桶也可能没有，有就拉到黄线上。改成：不设门槛一律校正；黄线也算赛道线（不要求有锥桶）；第一段三轮大平滑（配对门槛 1.2 → 0.9 → 0.7 m）拉直整体弯曲，第二段只认离线 0.3 m 内、且 2 m 内还有别的锥桶的，细校正后重画到线上（8 cm 实心圆），挤在 0.6 m 内的合并；赛道范围内其余黑点清掉，场外不动。",
                 "save_mapping 存完描黑后自动调用；map_report 结论表加“赛道拟合”一行和叠加图；一键脚本 -Save / -Report 加 -Track；手册 3.4.4。"],
        "结果": ["假数据（每种赛道加 0.1 m 噪声、15 个乱点、随机平移旋转、比例 0.97，两头翘 0.35 m 和 1.2 m 两档弯曲）：0.35 m 档 4/4 认对；1.2 m 档 3/4（D 认成 C，两者只差左头内圈，分数差 0.005，报告提示分不清）。锥桶都拉到线上；乱点里本来就落在赛道线 0.3 m 内的会被当锥桶收进来（真实地图杂物多在场外，影响小）。脚本见八、8.6.1。",
                 "第一版刚性拟合一开始就放开比例，外圈锥桶被吸到内圈线上，比例算成 0.946（真值 1.03）；改成先固定比例 1 再放开。第二版只做一轮校正时，15 个乱点里约 11 个被拉上线画成假锥桶；改成两段（先大平滑、再只认贴线的）。",
                 "11-03 操场练习场地六趟：分数 0.5~0.75（和规则差异大），强制校正后锥桶都在线上；21:23 那趟（两个里程计、重影）202 个点里 92 个拉上线、合并成 54 个，赛道内重影擦掉 73 个。六趟分别认成了 A/B/C/D——练习场地不是规则赛道，要固定形状就用 -Track。",
                 "没在真正按规则摆的赛道上测过；弯的精确坐标要规则原文。"],
    }),
    ("issue", "EKF 融合模式（-Odom ekf）：没有输入 / 航向自检对不上（2026-11-04）", {
        "现象": "-Odom ekf 建图时建图窗口出红字“没收到 /encoder、/IMU_data”“ekf_node 5s 内没有输出”，或“航向自检……比例 K”不在 0.8~1.25。",
        "原因": ["电脑上 robot_localization 融合 /encoder（twist.linear.x，m/s，transform_v.cpp 就这么用）和 /IMU_data（angular_velocity.z），这两个话题要经 WiFi 从车上来；车上驱动没起或电脑收不到就没有输入。",
                 "时间：小车每次开机时钟都不一样（用户 11-04 要求直接用小车时间，不换算）。ekf_node 默认按电脑时钟预测，直接给车上时间戳会全被当成“未来 / 过去的数据”；所以 ekf_input 把收到的车上时间戳发成 /ekf_clock，ekf_node 开 use_sim_time、/clock 改接 /ekf_clock，跟着小车时间走，输出 /ekf_odom 就是车上时间，建图桥直接按扫描时间戳插值。（第一版是换算成电脑时间再换回来，11-04 改掉。）",
                 "车上 /encoder、/IMU_data 的协方差都是 0，ekf_node 会当成“绝对准”，ekf_input 按参数填上。IMU 角速度单位 / 方向没在车上确认过，所以每 5 s 拿 /key1_odom 的航向变化对比。"],
        "解决": ["没有输入：小车上看两个话题在不在发、频率多少：",
                 C("# 小车\nsource /opt/ros/humble/setup.bash\nexport ROS_DOMAIN_ID=42\ntimeout 5 ros2 topic hz /encoder\ntimeout 5 ros2 topic hz /IMU_data\nros2 topic echo --once /IMU_data | head -n 30"),
                 "比例接近 57：IMU 发的是度/秒，按红字给的数加 -GyroScale（约 0.01745）；接近 −1：方向反了，加 -GyroScale -1；都不是：先别用 EKF 模式，把那行贴出来。",
                 "ekf_node 起不来、报 Failed to parse global arguments：ekf.yaml 里列表混写了整数和小数（0 和 0.05），rcl 解析失败直接退出；每个数都写成小数（0.0）。11-04 已改。"],
        "结果": "假数据冒烟（车上时钟快 1 年，3 m/s、0.5 rad/s 画圆，整套 launch odom_source:=ekf）：改用小车时间后 EKF 输出 599 条，时间戳就是车上时间（比电脑快 365.0 天），轨迹与半径 6 m 的圆最大差 0.05 m，弧长 35.9 m 与理论相同，航向自检比例 1.00。还没上车。",
    }),
    ("issue", "小车还没敲 1，电脑 rviz 里就有雷达数据（2026-11-03，待查）", {
        "现象": "小车新开 SSH 窗口、还没敲 1，电脑建图 rviz 里已经有红色扫描点、地图在长。",
        "排查": ["敲 1 启动的是 distributed_mapping_vehicle（11-04 起只有 imu_encoder_mix）。雷达 /scan 来自 Run_car.sh 里的雷达驱动，不靠敲 1，所以没敲 1 就有扫描是正常的；要查的是有没有上次敲 1 的里程计还在跑（窗口断开进程没退）。",
                 C("# 小车：看车上已经在跑什么\nps -ef | grep -E \"lslidar|imu_encoder_mix|distributed_mapping\" | grep -v grep")],
        "原因": "待用户贴上面命令的输出。",
        "解决": ["输出里有 distributed_mapping_vehicle / imu_encoder_mix：上次的还在，先在原窗口 Ctrl+C（或 kill 那几个进程号）再敲 1；直接再敲 1 会起第二个，两份 /key1_odom 打架（见“两个里程计同时在发”）。",
                 "只有雷达驱动：正常，敲 1 起里程计即可。"],
    }),
    ("issue", "描黑改成存图后处理；存图后描黑失败（2026-11-03）", {
        "现象": "建图时 rviz 里实时描黑的 Map Solid 跟着 SLAM 变、操场上看着乱，用户要求关掉，改成和 projectliu 一样：存好地图再描黑。",
        "解决": ["新增 mapping/scripts/solidify_map：读存好的 run1\\map.raw（autosave 没有 raw 就读 map.pgm），调 obstacle_layer.solidify()（算法不变），写 map_solid.* 和 solid.json，登记进 complete.json。离线没有扫描，半径固定 0.08 m（--radius 可改）。",
                 "save_mapping 存完原图、没收到 /map_solid 时自动调用；失败只打红字“描黑后处理失败（原图已存好，不受影响）”，原图不受影响。",
                 "launch 的 solid 默认改 false；一键脚本 -LiveSolid 才开实时（-NoSolid 保留但已无作用）；快捷键 2 = 建图（存图后描黑），4 = 建图 + 实时描黑。"],
        "结果": ["10-19 存好的 run1、11-03 20:02 的 autosave 离线描黑：68、41 个锥桶。按 3 保存时自动描黑还没在实车确认。",
                 "失败时：把红字整行贴出来；常见是 run1 里缺 map.yaml / map.pgm，或电脑上没同步到新的 obstacle_layer。原图能用，可以之后手动重做（手册 3.4.1 最后一个命令）。"],
    }),
    ("issue", "刚开机雷达不出数据：一直刷 SetupSerial 1: Inappropriate ioctl for device（2026-10-29，原因未证实）", {
        "现象": "室外约 7 ℃，开机后 Run_car.sh 里 lslidar 打印 open_port /dev/laser OK，然后一直刷 SetupSerial 1: Inappropriate ioctl for device；小车敲 1 后 scan_relay 每 10 s 报“转发扫描 0 帧”“没有收到 /scan”，里程计正常。约 6 分钟后正常。",
        "排查": ["/dev/laser -> ttyACM0（CH9102，1a86:55d4，cdc_acm，USB 口 3-1.2.4），规则 /etc/udev/rules.d/laser.rules 只按口位 KERNELS==\"3-1.2.4\"；用 460800 直接读 ttyACM0 是 a5 5a 开头的 M10 数据包 → 设备和链接都对，没插反。",
                 "其他口：car -> ttyUSB0（CH340，3-1.1.1）、encoder -> ttyUSB1（CH340，3-1.2.3）、IMU = ttyUSB2（CP2102N，3-1.2.2）。",
                 "所有串口设备节点时间同为 05:53、SSH 报 Network error: Software caused connection abort、车上时钟跳了约 1 天 → 那时小车很可能重启 / 掉电过，重启后就好了。"],
        "原因": "未证实。推测刚开机 USB 集线器供电不稳（冷天电池内阻大，雷达电机启动电流大），雷达串口掉线重连，驱动打开时串口没准备好且不会自己恢复。",
        "解决": ["临时：开机后等约 30 s、雷达在转，ls -l /dev/laser /dev/imu /dev/encoder /dev/car 四个都在再起 Run_car.sh；还刷 SetupSerial 就 Ctrl+C 等几秒重起，或只拔插雷达 USB。",
                 "下次复现先别重启，抓：dmesg -T | grep -iE 'usb 3-1|ttyACM|ttyUSB|disconnect|over-current|cdc_acm' | tail -40；看到反复的 USB disconnect / new full-speed USB device 就是供电掉线重连。"],
        "结果": "未解决，等复现时的 dmesg。",
    }),
    ("issue", "udevadm trigger 之后 /dev/imu 没了，里程计 0 条（2026-10-29）", {
        "现象": "为查雷达运行了 udevadm control --reload-rules && udevadm trigger，之后 ls 看不到 /dev/imu，敲 1 后 scan_relay 报“里程计 0 条”。",
        "原因": "IMU 的规则带 ACTION==\"add\"（只在插入时生效），udevadm trigger 默认发的是 change 事件，重新处理时没命中，链接被去掉。laser / car / encoder 的规则没有这个限制，不受影响。",
        "解决": ["按插入事件重新触发，或拔插 IMU 的 USB；然后重起 Run_car.sh（IMU 驱动只在启动时打开一次 /dev/imu）：",
                 C("# 小车\nudevadm trigger --action=add --subsystem-match=tty; sleep 2; ls -l /dev/imu /dev/laser /dev/encoder /dev/car")],
        "结果": "以后不要单独用不带 --action=add 的 udevadm trigger。",
    }),
    ("h2", "排错速查"),
    ("p", "下表带 [监控] 的红字、黄字都出现在电脑建图窗口（敲 2 或 4 的那个 PowerShell）里，每 15 秒刷新一次。"),
    ("table", ["现象", "处理"], [
        ["建图窗口一直报“5s 内没有收到 /key1_odom”", "小车没敲 1，或车上还是 11-04 之前的版本（发 /encoder_imu_odom）：按手册第 4 章上车"],
        ["-Odom ekf 红字“没收到 /encoder、/IMU_data”或“航向自检……对不上”", "见“EKF 融合模式”卡片"],
        [".\\run_slam_windows.ps1 找不到命令 / 禁止运行脚本", "前面加 .\\，或用 .bat（带 -ExecutionPolicy Bypass）"],
        ["红字“没连小车 WiFi”", "切到小车 WiFi（192.168.5.x）再运行"],
        ["-Check“发现的节点: 无”（已连小车 WiFi）", "两端域号要 42；别加 -AllNics；小车 Run_car 和里程计都启动了"],
        ["/map 发布者不为 0", "车上或别处已有 SLAM / map_server，先关掉"],
        ["-Check 提示有重名节点", "驱动启动了多次（常见 IMU talker 残留），清残留后用 Run_car_safe.sh"],
        ["地图歪、有重影", "先看报告“里程计是不是只有一个”；高速见“高速 3 m/s 建图”卡片（-Fast）；操场空地见“操场走长线地图偏移”卡片"],
        ["[监控] 红字“/key1_odom 有 N 个发布者”或“/key1_odom N 条/s”", "两个来源在发同一个话题：见“两个里程计同时在发”卡片，小车上 ros2 topic info /key1_odom -v 查是谁"],
        ["[监控] 地面线一直“还没认出”，rviz 车前有跟车走的直线", "车要开起来（沿车头方向开 1 m 以上）才认得出；还不行把那行和截图贴出来，或 -Ground fixed 直接给定"],
        ["存完原图后红字“描黑后处理失败”", "原图已存好；见“描黑改成存图后处理”卡片，之后用手册 3.4.1 的命令手动描黑"],
        ["保存提示“已存在，拒绝覆盖”", "用了旧目录：重新建图（会新建目录）或换 -RunIndex"],
        ["忘了保存就关了窗口", "看 <地图目录>\\autosave\\；.\\view_csv.bat -Map \"<地图目录>\\autosave\\map.yaml\""],
    ], [4.2, 6]),
])

# ------------------------------------------------------------------ 三、上传到小车
UPLOAD = dict(title="三、上传到小车：Run_car_safe", intro="为什么要安全版启动脚本，以及第一次怎么验证。", blocks=[
    ("issue", "IMU_publisher 重复、驱动残留", {
        "现象": "-Check 发现重名节点；ros2 node list 里 IMU_publisher 出现两次。",
        "原因": "Run_car.sh 停止后留下孤儿 hipnuc_imu/talker 等进程；SSH 断开会杀掉手动 launch 的 imu_encoder_mix；驱动重启时里程计要一起重启。",
        "解决": "Run_car_safe.sh：启动前清掉残留的 talker、lslidar_driver_node、encoder_node；已有 Run_car 在跑就拒绝重复启动；Ctrl+C 先正常收尾，再对整个进程组扫尾强杀，15 秒看门狗兜底。",
        "结果": "只在电脑上做过语法检查，Git Bash 模拟不了 Linux 进程组，没有在小车上实测。",
    }),
    ("h2", "第一次试用的验证"),
    ("code", HEAD + "\nbash /home/racecar/Run_car_safe.sh"),
    ("code", r"""# 小车另一个窗口：只按一次 Ctrl+C、看到“所有进程已终止”后检查，应该没有任何输出
pgrep -ax talker; pgrep -af lslidar_driver_node; pgrep -af encoder_node
# 再启动一次后应为 1
ros2 node list | grep -c IMU_publisher"""),
    ("issue", "开机自启 car-odom.service 导致启停顺序不对（假设，未证实）", {
        "解决": "已删除小车 systemd 开机自启，改回手动：先 Run_car，等约 10 秒，再启动里程计（快捷键 1）。",
    }),
])

# ------------------------------------------------------------------ 四、CSV
CSV = dict(title="四、CSV 路径与地图融合", intro="调试 view_path_on_map / view_csv 时修过的坑。", blocks=[
    ("issue", "rviz 视角加载失败", {"原因": "视角类名写成 TopDownOrthoView。", "解决": "改为 rviz_default_plugins/TopDownOrtho。"}),
    ("issue", "节点启动即退出：参数类型不对", {"原因": "-p dx:=0 被 ROS 当成整数，参数声明是小数。", "解决": "脚本里统一格式化为 0.0（InvariantCulture）。"}),
    ("issue", "rviz2 报 Frame [map] does not exist，什么都不画", {"原因": "离线时没有 SLAM 发 TF。", "解决": "节点自己发静态 odom→map 单位变换（publish_tf；真 SLAM 在跑时设 false）。"}),
    ("issue", "空参数 -p name:= 解析失败", {"原因": "Windows 这套 ROS 不接受空值。", "解决": "脚本只在有值时才传该参数。"}),
    ("issue", ".ps1 里的中文路径乱码", {"原因": "Windows PowerShell 5.1 按 ANSI 读无 BOM 的文件。", "解决": "含中文的 .ps1 保存为 UTF-8 带 BOM。"}),
    ("note", "已验证：节点在 D:\\ubuntu\\slam 能运行，发布 path.csv 673 点和调试段，rviz2 订阅了各话题。未验证：Ubuntu 上 ros2 launch 路径。节点日志：%TEMP%\\view_csv_node.err。"),
])

# ------------------------------------------------------------------ 五、AB
AB = dict(title="五、AB 路径拼接", intro="代码检查后的修正，以及已知限制。", blocks=[
    ("table", ["位置", "问题", "修正"], [
        ["get_local_path.cpp AB_stop::concat_path", "AB 第一个点 (0,0,0) 和分叉点重合，零长度线段处曲率算成 0，前馈跳一下", "跳过离原点 < 1 cm 的点"],
        ["get_local_path.cpp 构造函数", "path.csv 结尾常多存一个相同点", "读入后去掉相距 < 1 cm 的连续重复点，日志打印去掉几个"],
        ["get_local_path.cpp AB_stop::load", "只录一边时读不存在的文件，打印“无法打开CSV文件”像出错", "先 std::filesystem::exists，不存在直接镜像"],
        ["get_local_path.cpp ab_select_callback", "运行中切换后路径变短，last_index 可能越界", "重建后把 last_index 限制在新路径范围内"],
        ["save_ab_path.cpp", "Ctrl+C 时最后一段（< save_dx）没存，停车点少最多 10 cm", "析构时补存最后位置"],
        ["save_ab_path.cpp", "构造函数里 shutdown 后 main 又 spin + shutdown", "构造失败标记 ok_=false，main 判断后再 spin"],
        ["save_ab_path.cpp", "M_PI 在部分编译器下未定义", "改用常量 AB_PI"],
    ], [3, 4, 3.5]),
    ("note", "已验证：save_ab_path.cpp 用桩头文件通过 g++ -fsyntax-only -Wall -Wextra；AB_stop + CSV_get_Path 单独编译运行，只录 A 时 B 自动镜像、旋转后落在正确一侧、分叉点不重复、AB 段速度 1.5。完整节点还没在小车上编译。"),
    ("h2", "已知限制（未改）"),
    ("ul", [
        "AB 段朝向基准只用分叉点一个点的 yaw，有噪声时 AB 段整体有小角度偏差；可用 concat_dy 微调，或改成用分叉点前几个点的连线方向。",
        "slow_point（15 点约 1.5 m）比 AB 段长时，减速会从分叉点之前开始，属正常。",
        "当前 path.csv（673 点）末点 (-5.47, -0.27)，最后两行重复，按流程重录后会被替换。",
    ]),
    ("issue", "室外跑路径（move_test）发了指令车一直不动，回室内立刻正常（2026-10-29，原因未证实）", {
        "现象": "室外约 7 ℃：手柄录轨迹正常（IMU、里程计好），雷达后来也正常、能建图；但启动 move_test 跑路径后车一直不动。回室内全部正常。",
        "排查": ["运动链路全在车上：/dev/imu→talker /IMU_data + /dev/encoder→encoder_node → imu_encoder_mix /encoder_imu_odom → get_local_path（只在里程计回调里发 /my_local_path）→ pure_track_test（只在路径回调里发 cmd_vel，没有定时兜底）→ racecar_driver_node → /dev/car。WiFi 不在链路里。",
                 "代码（电脑 aeb9085 版；小车上 get_local_path.cpp、move_test.launch.py 有未提交改动，可能不同）：find_nearest_index 从 last_index 只往前找、不回头；局部路径首点 v == -2（路径最后 stop_point=5 个点）时 pure_track_test 打印一次 is stop、end_flag=1，之后再也不发 cmd_vel，只能重启节点。"],
        "原因": ["未证实，三个嫌疑：① 两个 imu_encoder_mix 同时发 /encoder_imu_odom（快捷键 1 建图、record_seg.py 录轨迹、move_test 的 start_odom:=true 各起一个），位姿来回跳 → 下标滑到末尾 → 永久停车；",
                 "② SSH 断开（室外 WiFi 弱，10-29 确实断过）→ 前台的 Run_car.sh / move_test 收到挂断信号一起退出；",
                 "③ 里程计没归零（imu_encoder_mix 早起、车被推过）。"],
        "解决": ["规避：跑路径前先 Ctrl+C 停掉建图（敲 1 的窗口）和录轨迹窗口；在 tmux 里起 Run_car.sh 和 move_test（tmux new -s car，断线后 tmux attach -t car）。",
                 "复现时先别停，在小车新窗口抓（顺序看第一个没数据的环节）：",
                 C("# 小车\nsource /opt/ros/humble/setup.bash\nros2 topic info /encoder_imu_odom          # Publisher count 应为 1\nros2 node list | grep -c imu_encoder_mix    # 应为 1\npgrep -af \"racecar_driver_node|pure_track_test|get_local_path|imu_encoder_mix\"   # 4 个都在\nfor t in /IMU_data /encoder_imu_odom /my_local_path /cmd_vel; do echo \"== $t\"; timeout 4 ros2 topic hz $t | tail -2; done\ntail -3 $(ls -t /home/log/lateral_*.csv | head -1)   # 最后一列 END=永久停车 WAIT=等灯/斑马线 RUN=在发指令"),
                 "代码防护（运动代码，用户同意后才改）：最近点只在当前位置附近一小段窗口里找；只有真的走到终点附近才认停车点。"],
        "结果": "未解决，等复现时的输出。",
    }),
])

# ------------------------------------------------------------------ 六、视觉
VISION = dict(title="六、红绿灯视觉（本地 / 远程）", intro="两套共用摄像头和话题名，不能同时运行。", blocks=[
    ("table", ["现象", "处理"], [
        ["找不到 output/yolov5n_bs1.om", "先转换或 scp 模型（手册第 7、9 章）"],
        ["摄像头 /dev/video0 被占用", "关掉占用的程序（record_video.py、start_vision.sh、远程识别），fuser -v /dev/video0 确认"],
        ["light_infer.py 已经在运行", "pgrep -f light_infer.py 找到并停掉"],
        ["识别进程启动后退出", "看 infer.log 最后几行"],
        ["话题没有数据 / No module named rclpy", "ROS 部分用 /usr/bin/python3，别用 conda 的 python3"],
        ["can't open file '...\\pc_vision_client.py'", "先 cd 到 camera_capture 目录"],
        ["连不上 192.168.5.100:9000", "小车端没启动或已退出；电脑不在 192.168.5.x；开了 VPN"],
        ["PowerShell 不让运行 .ps1", "powershell -ExecutionPolicy Bypass -File ...，或直接运行 pc_vision_client.py"],
        ["/traffic_state 一直 stale", "电脑端没连上或结果超时；看小车上运行 start_remote_vision.sh 的那个终端里有没有打印“电脑已连接”"],
        ["topic does not appear to be published yet", "小车端没启动、没 source，或跨机看话题时两端 ROS_DOMAIN_ID 不一致"],
        ["电脑画面卡顿、延迟大", "小车端 --fps 10 --quality 70；离路由器近一些"],
    ], [4, 6]),
    ("issue", "远程识别状态在 stale 和正常之间来回跳（2026-10-11 实车）", {
        "现象": "往返延迟 p50 约 70–110 ms，但偶发 p95 冲到约 3.4 秒，/traffic_state 闪 stale。",
        "原因": "没查清，可能是 WiFi 抖动。",
        "解决": ["持续 ping 排查；接控制前考虑把 --stale-sec 放宽到 0.8 或 1.0。", C("# 电脑 PowerShell\nping -t 192.168.5.100")],
    }),
    ("issue", "小车上的视觉改动没同步到电脑（未完成）", {
        "现象": "小车上 light_bridge.py、start_vision.sh 有未提交改动，还多了 light_infer_daemon.py。",
        "解决": [C("# 小车：生成补丁\ncd /home/racecar/src/my_racecar && git --no-pager diff -- camera_capture/light_bridge.py camera_capture/start_vision.sh > /tmp/car_vision_changes.patch && cp camera_capture/light_infer_daemon.py /tmp/ && wc -c /tmp/car_vision_changes.patch /tmp/light_infer_daemon.py"),
                 C("# 电脑 PowerShell：拷回桌面\nscp root@192.168.5.100:/tmp/car_vision_changes.patch \"$env:USERPROFILE\\Desktop\\car_vision_changes.patch\"\nscp root@192.168.5.100:/tmp/light_infer_daemon.py \"$env:USERPROFILE\\Desktop\\light_infer_daemon.py\""),
                 "看清内容，确认和 --enable-control 是否重复或冲突，再走标准同步流程提交。"],
    }),
    ("note", "远程识别已在电脑回环验证：状态随画面变化（none → red → green），往返 45–60 ms，断开后立即 stale。--enable-control 的 ROS 路径只验证过语法。"),
])

# ------------------------------------------------------------------ 七、训练
TRAIN = dict(title="七、红绿灯模型训练与转 .om", intro="采集、标注、训练、转换过程中的问题。", blocks=[
    ("table", ["问题", "原因", "处理"], [
        ["车上 record.py 录不了视频", "它订阅 /camera1/image_raw，车上没有在发", "自己写 record_video.py 直接读 USB 摄像头"],
        ["抽帧时 OpenCV 读不到图", "cv2.imread 不支持中文路径", "cv2.imdecode(np.fromfile(...))"],
        ["导入 torch 很慢像卡住", "第一次编译缓存、杀毒扫描", "等 1–2 分钟，别按 Ctrl+C"],
        ["椅子、人被框成灯箱", "训练里没有负样本", "几何过滤 + 更高置信度；根本办法是补负样本重训"],
        ["绿灯认不出来", "最早的数据没有绿灯", "补拍绿灯、红变绿视频重训"],
        ["灯箱切在画面边缘被标成“没亮”", "可能只露出半个灯", "标注时剔除"],
        ["有亮灯但没框到灯箱，被当负样本", "自动标注没找到灯箱", "这类帧一律剔除"],
        ["橙灯定位的灯箱框偏", "框到橙灯附近或红锥桶上", "灯箱模型的框优先，橙灯定位作后备"],
        ["YOLO11 模型不能上车", "课程推理按 YOLOv5 v7.0 输出格式写", "用 YOLOv5 v7.0 训练 YOLOv5n"],
        ["ONNX 可能转不了", "v7.0 默认 opset 17", "导出 --opset 12，不要 --dynamic"],
        ["车上转 .om 慢且可能失败", "课程记录约 2 小时；车上内存约 3.4 GB", "Linux 电脑用 CANN 7.0.RC1 x86_64 转，soc 写 Ascend310B4"],
        ["车上 conda python3 导入不了 rclpy", "rclpy 是系统 Python 3.10 的", "ROS 脚本用 /usr/bin/python3"],
        ["车上不能 pip install", "没有外网", "电脑下载好再 scp"],
        ["小车压力太大，跑不了行驶程序", "NPU 推理、torch、CPU 上的 NMS，加车上 rviz2（推断，没量过）", "改成电脑识别（手册第 8 章）"],
    ], [3.2, 3.4, 3.8]),
    ("issue", "电脑摄像头试验发现的问题", {
        "现象": ["椅子、人被框成灯箱；红灯亮时灯箱框反而出不来（不亮时灯箱分数 0.57，红灯光晕把分数拉到阈值以下，亮灯框必须在灯箱里的规则又把真红灯否掉）。"],
        "原因": "训练数据只有一间教室、一块灯箱，负样本只有约 6 张，没有三灯全灭和橙灯亮。笔记本摄像头和小车摄像头不同。",
        "解决": "降低 --conf-box 可缓解；要补拍负样本（100–200 张椅子、人、桌子、锥桶、屏幕）、不同距离角度光线，再重标重训。",
    }),
    ("h2", "转换常见问题"),
    ("table", ["现象", "处理"], [
        ["atc: command not found", "没有 source ~/Ascend/ascend-toolkit/set_env.sh"],
        ["安装 CANN 报 Python 版本或缺包", "换 Python 3.9 环境，装 attrs numpy decorator sympy cffi pyyaml pathlib2 psutil protobuf scipy requests absl-py"],
        ["转换报算子不支持", "用 --opset 11 重新导出 ONNX"],
        ["车上加载 .om 报版本不匹配", "电脑 CANN 要 7.0.RC1，innerversion 最好也是 V100R001C13SPC005B246"],
        ["转换时路径报错", "目录和文件名用纯英文"],
    ], [3.5, 6.5]),
])

# ------------------------------------------------------------------ 八、调试命令
DEBUG = dict(title="八、调试命令", intro="验证、自检用的命令，按模块分。手册里只放最终运行的命令。", blocks=[
    ("h2", "8.1 git：核对两边是否一致"),
    ("code", r"""# 小车：看提交和未提交的改动
cd /home/racecar/src/my_racecar && git log --oneline -5 && git status --short"""),
    ("code", r"""# 电脑 PowerShell：和小车已提交内容的差异（看不到小车未提交的改动）
cd "C:\Users\lzc2\Desktop\my_racecar（复件）"; git fetch car; git diff --stat main car/main"""),
    ("code", r"""# 整个项目哈希（两边结果相同 = 一致；只覆盖 git 跟踪的文件）
# 小车
cd /home/racecar/src/my_racecar && git ls-files -z | xargs -0 git hash-object | sha256sum
# 电脑 Git Bash（不能用 PowerShell）
cd "/c/Users/lzc2/Desktop/my_racecar（复件）" && git ls-files -z | xargs -0 git hash-object | sha256sum"""),
    ("h2", "8.2 建图：小车自检"),
    ("code", HEAD + "\ntimeout 5 ros2 topic hz /scan\ntimeout 5 ros2 topic hz /key1_odom\nros2 topic info /key1_odom -v | grep -E \"Node name|Publisher count\""),
    ("p", "/scan 应稳定约 10 Hz；/key1_odom 约 50 Hz（车停、车动都一样），Publisher count: 1。车动时 /key1_odom 到 90 Hz 左右、或 Publisher count: 2：两个里程计，见二“两个里程计同时在发”。"),
    ("h2", "8.3 建图：Windows 联机检查（约 8 秒）"),
    ("code", r"""# 电脑 PowerShell
cd D:\ubuntu\slam
.\run_slam_windows.bat -Check"""),
    ("p", "通过：发现的节点列出小车节点；/scan 约 10 Hz、/key1_odom 约 50 Hz 都收到，/key1_odom 发布者 1；/map 发布者 0；没有重名节点。要用 -Odom ekf 时 /encoder、/IMU_data 发布者都应为 1。"),
    ("h2", "8.4 建图：Linux 联机检查"),
    ("code", r"""# Linux 电脑
cd "/data/code/智能车/智能车(1)/my_racecar（复件）(1)"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
timeout 6 ros2 topic hz /scan
timeout 6 ros2 topic hz /key1_odom
ros2 topic info /scan -v | head -n 12"""),
    ("h2", "8.5 建图：看日志判断漂移"),
    ("p", "<地图目录>\\.run1-live.jsonl 每 5 秒一行：scans、odometry_messages、known_cells、odom / slam / corr 三组 [x, y, yaw°]；event 为 stalled 表示 30 秒没增长。判读方法见二、分布式建图“第二圈漂移”卡片。"),
    ("p", "同一文件每行的 link 字段：odom_pubs（车上 /key1_odom 发布者个数）、odom_rate（建图桥数的 /key1_odom 条数 / s，一个约 50）、ground、ground_removed。下面这条把 11 月每趟按“车动时里程计条数、有没有两个、corr 最大跳”汇总成一行（11-04 用它查出两个里程计）："),
    ("code", r"""# 电脑 PowerShell
cd D:\ubuntu\slam
$env:PYTHONIOENCODING = 'utf-8'
pixi run --frozen python "D:\VSCode\ros2技术集\智能车总文件\my_racecar（复件）\事项\临时修改\windows建图linux建图分布式通信\windows\模拟测试\高速建图\live_sum.py" """),
    ("p", "每行：趟名、车动的 5 s 段数、其中两个里程计的段数及“假车速”和 corr 最大跳、一个里程计时的车速和 corr 最大跳、条数 / s。车动时 70 条/s 以上就是两个。"),
    ("h2", "8.5.1 实体化障碍层：看 solid.json"),
    ("p", "<地图目录>\\.run1-solid.json（建图中每秒更新）或 run1\\solid.json（保存后）：cone_radius 画图半径、radius_samples 单帧半径样本数（少于 20 用默认 0.08）、"
          "cones 锥桶数、not_isolated 因外环有障碍或周围没探明而没画的点团数、moved_tracks 判为会移动的短段数、tf_miss 查不到里程计位姿的帧数（应接近 0）、"
          "cone_centers 每个圆 [x, y, 原图点团格数]。"),
    ("code", r"""# 电脑 PowerShell：看最新一次的记录
$f = (Get-ChildItem D:\ubuntu\slam\maps -Recurse -Filter solid.json | Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName
Get-Content $f -Encoding UTF8"""),
    ("p", "锥桶漏画：先看原图 map.png 那里有没有点（没有就补不出来）；有点却没画，多半是离边缘不到 edge_margin（1.0 m）、外环（锥桶边缘外 0.08~0.25 m）里别的团超过 ring_max（2 格），"
          "或外环空闲比例低于 ring_free_min（0.6）。参数默认值在 obstacle_layer 的 __init__ 里。离线试参数：用保存的 map.pgm 跑 solidify 函数即可，不用上车，命令如下。"),
    ("code", r"""# 电脑 Git Bash：离线复查一张图（没按 3 保存时用 <地图目录>/autosave/map.pgm）
cd "/d/VSCode/ros2技术集/智能车总文件/my_racecar（复件）/事项/临时修改/windows建图linux建图分布式通信/windows/模拟测试/实体层"
PYTHONIOENCODING=utf-8 python solid_diff.py "C:/Users/lzc2/Desktop/my_racecar（复件）/mapping/scripts/obstacle_layer" /d/ubuntu/slam/maps/<地图目录>/run1/map.pgm out.png <半径> 3 '{"edge_margin": 1.0}'"""),
    ("p", "半径取 solid.json 的 cone_radius。输出 out.png：灰度是原图，红色是实体层新加的，蓝色是被清掉的原图障碍；终端打印锥桶数和没加工的小团数。"
          "同目录的 edge_diag.py 用来查某个位置离边缘、离别的点团有多远。"),
    ("h2", "8.6 建图：无车模拟（三个 PowerShell 窗口）"),
    ("code", r"""# 窗口一：SLAM
cd D:\ubuntu\slam
$env:ROS_LOCALHOST_ONLY="1"; $env:ROS_DOMAIN_ID="77"
$t = Get-Date -Format "yyyyMMdd_HHmmss"
pixi run --frozen ros2 launch "C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\launch\distributed_mapping_pc_windows.launch.py" map_dir:=D:/ubuntu/slam/maps_test_$t rviz:=false"""),
    ("code", r"""# 窗口二：模拟小车约 24 秒（脚本在 临时修改\windows建图linux建图分布式通信\windows\模拟测试\）
cd D:\ubuntu\slam
$env:ROS_LOCALHOST_ONLY="1"; $env:ROS_DOMAIN_ID="77"
pixi run --frozen python "<模拟测试目录>\fake_car_模拟小车(可加noodom参数).py" 24"""),
    ("code", r"""# 窗口三：保存测试地图
cd D:\ubuntu\slam
$env:ROS_LOCALHOST_ONLY="1"; $env:ROS_DOMAIN_ID="77"
$d = (Get-ChildItem D:\ubuntu\slam -Directory -Filter "maps_test_*" | Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName
pixi run --frozen python "C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\scripts\save_mapping" --run-index 1 --map-dir ($d -replace '\\','/')"""),
    ("h2", "8.6.1 建图：高速建图测试（去畸变、EKF、录包对比重建）"),
    ("p", "脚本在 临时修改\\windows建图linux建图分布式通信\\windows\\模拟测试\\高速建图\\（抗丢包的模拟脚本在旁边的 抗丢包\\，功能 11-04 已删，脚本只留作记录）。"),
    ("code", r"""# 电脑 PowerShell：去畸变假数据测试（约 2 秒，不用开 ROS 节点）
cd D:\ubuntu\slam
$env:PYTHONIOENCODING = 'utf-8'
pixi run --frozen python "D:\VSCode\ros2技术集\智能车总文件\my_racecar（复件）\事项\临时修改\windows建图linux建图分布式通信\windows\模拟测试\高速建图\test_deskew.py" """),
    ("p", "正常：三行，“去畸变 stamp_at_end=true”那行中位误差约 0.003 m，比“不去畸变”（约 0.08 m）和“设反”（约 0.2 m）都小得多。"),
    ("code", r"""# 电脑 PowerShell：EKF 冒烟（整套建图 launch odom_source:=ekf + 假小车，约 30 秒，本机 79 号域）
powershell -ExecutionPolicy Bypass -File "D:\VSCode\ros2技术集\智能车总文件\my_racecar（复件）\事项\临时修改\windows建图linux建图分布式通信\windows\模拟测试\高速建图\fake_launch.ps1" """),
    ("p", "正常：先列出 /ekf_filter_node、/ekf_input、/scan_tf_bridge、/slam_toolbox 等节点，再打印 EKF 输出约 600 条、时间戳差约 365 天、离圆的最大差 < 0.1 m、弧长两个数相同。命令跑完不返回时，是 ros2 daemon 占着输出：任务管理器结束 python（命令行带 ros2-daemon 的那个）。"),
    ("code", r"""# 电脑 PowerShell：用车上录的包对比重建（-Mapper 换参数文件、-NoDeskew 关去畸变；本机 77 号域，不连小车）
powershell -ExecutionPolicy Bypass -File "D:\VSCode\ros2技术集\智能车总文件\my_racecar（复件）\事项\临时修改\windows建图linux建图分布式通信\windows\模拟测试\高速建图\replay.ps1" -Bag D:\ubuntu\slam\bags_car\<包目录> -Out D:\ubuntu\slam\maps_replay\<包目录>_fast -Mapper "C:\Users\lzc2\Desktop\my_racecar（复件）\mapping\config\mapper_fast.yaml" """),
    ("p", "只用来对比参数（不是建图功能，抗丢包的 -Bag 已删）：包里两个里程计混在一起时，player.py 按位姿连续性分成几条轨迹、只放最长的一条（--track 换）。正常：最后打印“建图报告：<Out>\\run1\\report.html”。"),
    ("code", r"""# 电脑 PowerShell：赛道拟合假数据测试（4 种赛道各生成一份，约 20 秒）
cd D:\ubuntu\slam
$env:PYTHONIOENCODING = 'utf-8'
pixi run --frozen python "D:\VSCode\ros2技术集\智能车总文件\my_racecar（复件）\事项\临时修改\windows建图linux建图分布式通信\windows\模拟测试\高速建图\test_fit_track.py" """),
    ("p", "正常：8 行（两头翘 0.35 m、1.2 m 各 4 种），最后一行“结论：7/8 认对”或 8/8（1.2 m 档 C/D 可能互认错，两者只差左头内圈）；每行“拉到线上”个数接近“真锥桶”个数。每行最后给的 track_fit.png 可以打开看。"),
    ("h2", "8.6.2 建图：地面线过滤模拟测试"),
    ("p", "test_ground.py 直接调用 scan_tf_bridge 里的 GroundLineFilter，模拟 5 种情况：操场 1.5 m/s、0.3 m/s（应认出），走廊两种、静止（不应认出）。约 20 秒，不用开 ROS。"),
    ("code", r"""# 电脑 PowerShell
cd D:\ubuntu\slam
$env:PYTHONIOENCODING = 'utf-8'
pixi run --frozen python "D:\VSCode\ros2技术集\智能车总文件\my_racecar（复件）\事项\临时修改\windows建图linux建图分布式通信\windows\模拟测试\地面线\test_ground.py" """),
    ("p", "正常：最后一行“结论： 全部符合预期”；A、A2 打印“第 N 帧认出：theta≈2.0° rho≈3.00 m”和“锥桶点保留”接近全部；B、B2、C 打印“没认出地面线”。"),
    ("h2", "8.7 视觉：摄像头和话题"),
    ("code", r"""# 小车：环境检查（只读）、模型和摄像头占用（fuser 没输出 = 空闲）
cd /home/racecar/src/my_racecar/camera_capture && bash check_env.sh
cd /home/racecar/src/my_racecar/camera_capture && ls -lh output/yolov5n_bs1.om && fuser -v /dev/video0"""),
    ("code", r"""# 小车：看话题
source /opt/ros/humble/setup.bash && ros2 topic echo /traffic_state
source /opt/ros/humble/setup.bash && ros2 topic hz /traffic_state"""),
    ("code", r"""# 小车：只调识别、不发话题
cd /home/racecar/src/my_racecar/camera_capture && bash run.sh          # 终端打印状态
bash run.sh video        # 录 30 秒带框视频到 /tmp/out.mp4
bash run.sh images 文件夹   # 离线跑图片"""),
    ("h2", "8.8 视觉：远程识别"),
    ("code", r"""# 小车：确认文件版本（MD5 应为 aa0f449e... / abc2589c... / 21faefef...）
cd /home/racecar/src/my_racecar/camera_capture && git log --oneline -1 && md5sum remote_vision_bridge.py remote_proto.py start_remote_vision.sh && fuser -v /dev/video0
# 小车：只测负载，不连电脑
cd /home/racecar/src/my_racecar/camera_capture && bash start_remote_vision.sh --selftest
top -b -n 1 | head -15"""),
    ("code", r"""# 电脑 PowerShell：端口通不通（TcpTestSucceeded : True）
Test-NetConnection 192.168.5.100 -Port 9000"""),
    ("table", ["验证项", "通过标准"], [
        ["连通", "小车上运行 start_remote_vision.sh 的终端里出现“电脑已连接”，往返 p50 约 70–110 ms"],
        ["话题", "ros2 topic hz /traffic_state 约 20 Hz"],
        ["断线", "关掉电脑端，0.5 秒内变 stale，重开自动恢复"],
        ["红判绿", "对红灯看一段时间，不出现 green"],
    ], [2, 8]),
    ("h2", "8.9 模型：检查 .om 并在车上加载"),
    ("code", r"""# 电脑 PowerShell（在 yolov5n11n.om训练过程\ 下）：看 .om 里嵌的转换参数
python 04_转om\check_om.py 06_模型文件\yolov5n_bs1.om
# 电脑摄像头实时试 YOLO11n
D:\light_ai\venv\Scripts\python.exe 05_验证\light_yolo11_cam.py --model 06_模型文件\yolo11n_best.pt"""),
])

CHAPTERS = [GIT, MAP, UPLOAD, CSV, AB, VISION, TRAIN, DEBUG]
