# -*- coding: utf-8 -*-
"""《代码改动记录》内容：按模块列出新增 / 修改的文件，以及 git 提交。
以后每次改代码：在 LOG 末尾加一行；文件补进对应模块的表。
路径相对项目根（电脑 C:\\Users\\lzc2\\Desktop\\my_racecar（复件）\\，小车 /home/racecar/src/my_racecar/），另有注明的除外。
"""
UPDATED = "2026-11-10"

INTRO = dict(title="一、说明", intro="怎么看这份记录。", blocks=[
    ("ul", [
        "路径相对项目根：电脑 C:\\Users\\lzc2\\Desktop\\my_racecar（复件）\\，小车 /home/racecar/src/my_racecar/。D:\\ubuntu\\slam\\ 和小车 /home/racecar/ 下的文件单独注明。",
        "A = 新增，M = 修改。提交号是 git 提交的前 7 位；“未提交”表示只在电脑工作区。",
        "临时修改\\ 下各模块文件夹里是这些文件的副本（备份），以项目里的为准。",
        "具体改了哪些代码看同目录的 代码改动记录.html（双击用浏览器打开）：大模块 → 功能 → 文件，每个模块和功能写明要求、实现、验证状态，每个文件旁写它的作用；增 / 删 / 改都能展开看代码，显示的是功能最终完成时相对开始前的净改动，未验证的功能继续跟进。"
        "内容写在 文档\\源\\features.py；规范见 C:\\Users\\lzc2\\Desktop\\AI约束文件\\代码修改记录网页生成\\。",
    ]),
    ("code", r"""# 电脑 PowerShell：看某次提交改了哪些文件
cd "C:\Users\lzc2\Desktop\my_racecar（复件）"
git show --stat 10b04bd"""),
])

LOG = dict(title="二、改动总表", intro="按时间正序。", blocks=[
    ("table", ["日期", "提交", "改动", "文件"], [
        ["09-29", "d160d20 / 5bfdfd5", "分布式建图初版、Ubuntu 说明", "A mapping/DISTRIBUTED.md 等"],
        ["10-01", "c587ccd", "scan_tf_bridge 修复 SLAM 不出图，建图 launch 集中到 mapping/", "A mapping/scripts/scan_tf_bridge\nM mapping/launch/distributed_mapping_pc.launch.py、mapper.yaml"],
        ["10-05", "b75625f", "红绿灯识别（16 个文件）", "A camera_capture/…"],
        ["10-05", "10b04bd", "建图：自动保存、告警、锥桶阈值、Windows 入口、查看工具", "A mapping/scripts/map_autosave、view_path_on_map\nA mapping/launch/distributed_mapping_pc_windows.launch.py、view_path_on_map.launch.py\nA scripts/Run_car_safe.sh\nM scan_tf_bridge、mapping_status_logger、mapper.yaml、CMakeLists.txt"],
        ["10-05", "02a512a（小车）", "红绿灯接入运动控制", "M pure_track_test 等（light_mode、/traffic_light_req）"],
        ["10-05", "8daf9c8 / 641ae59", "合并小车；远程视觉", "A camera_capture/remote_vision_bridge.py、remote_proto.py、start_remote_vision.sh、pc_vision_client.py、start_pc_vision.ps1"],
        ["10-11", "未提交", "建图漂移：惩罚、回环参数；记录位姿", "M mapping/config/mapper.yaml、mapper_native.yaml\nM mapping/scripts/mapping_status_logger"],
        ["10-17", "未提交", "保存时多存 map.png", "M mapping/scripts/save_mapping"],
        ["10-17", "（不在仓库）", "快捷键", "M 小车 /root/.bashrc\nA 电脑 Documents\\WindowsPowerShell\\Microsoft.PowerShell_profile.ps1"],
        ["10-19", "未提交", "实体化障碍层 /map_solid", "A mapping/scripts/obstacle_layer\nM mapping/launch/distributed_mapping_pc_windows.launch.py、distributed_mapping_pc.launch.py（solid 参数）\nM mapping/rviz/slam_mapping.rviz（Map Solid）\nM mapping/scripts/save_mapping（存 map_solid.*、solid.json）\nM CMakeLists.txt（install obstacle_layer）\nM D:\\ubuntu\\slam\\run_slam_windows.ps1（-NoSolid）\nM 电脑 PowerShell 配置：2 = 不开实体层，新增 4 = 开实体层"],
        ["10-21", "未提交", "抗丢包建图：车端可靠补发、电脑排队补帧、sync SLAM、车上录包、离线重建", "A mapping/scripts/scan_relay\nM mapping/scripts/scan_tf_bridge（source、排队区、去重、/odom_pc、/scan_link）\nM mapping/scripts/mapping_status_logger（可选话题参数、link 字段）\nM mapping/launch/distributed_mapping_vehicle.launch.py（relay、record 参数）\nM mapping/launch/distributed_mapping_pc_windows.launch.py、distributed_mapping_pc.launch.py（source、slam_mode）\nM CMakeLists.txt（install scan_relay）\nM D:\\ubuntu\\slam\\run_slam_windows.ps1（-Bag、-Rate、-Async、[监控] 网络、-Check 查 /scan_relay）"],
        ["10-21", "未提交", "代码改动记录网页版：按模块 → 功能，写要求 / 实现 / 状态 / 文件作用，增删改都显示代码", "A 文档\\源\\diffs.py（生成 代码改动记录.html，快照存 文档\\源\\改动快照\\）\nA 文档\\源\\features.py（模块、功能、文件说明）\nM 文档\\源\\build.py（最后运行 diffs）\nM CLAUDE.md（文档规则加一条）\nA 桌面 AI约束文件\\代码修改记录网页生成\\代码修改记录网页生成规范.md"],
        ["10-22", "eac0063（小车）/ aeb9085", "拉回小车红绿灯按需推理、AB 路径；建图改动（抗丢包 + 实体层）提交并上车；文档平时只更新网页", "eac0063：camera_capture/light_bridge.py、light_infer_daemon.py、light_web_btn.py、start_vision.sh，include/*.csv，launch/move_test.launch.py，scripts/record_seg.py，src/get_local_path.cpp（小车提交，快进合并）\naeb9085：上面 10-11 ~ 10-21 两行的 mapping/*、CMakeLists.txt（scan_relay、obstacle_layer、scan_tf_bridge 设可执行）\nM 文档\\源\\build.py（默认只生成网页，--pdf 才生成 PDF）"],
        ["11-03", "未提交", "操场长线偏移：匹配 / 回环参数改回默认；scan_tf_bridge 地面线过滤；描黑改成存图后处理", "M mapping/config/mapper.yaml、mapper_native.yaml（搜索 ±0.25 m / ±20°，惩罚、回环阈值改回默认）\nM mapping/scripts/scan_tf_bridge（GroundLineFilter，ground_filter / ground_rho / ground_theta_deg）\nA mapping/scripts/solidify_map（存好的地图离线描黑）\nM mapping/scripts/save_mapping（没有 /map_solid 时存完自动调 solidify_map）\nM mapping/launch/distributed_mapping_pc_windows.launch.py、distributed_mapping_pc.launch.py（ground_* 参数；solid 默认 false）\nM CMakeLists.txt（install solidify_map）\nM D:\\ubuntu\\slam\\run_slam_windows.ps1（-Ground、-GroundRho、-GroundDeg、-LiveSolid，[监控] 地面线一行）\nM 电脑 PowerShell 配置：2 = 建图（存图后描黑），4 = 建图 + 实时描黑"],
        ["11-03", "未提交", "建图报告：每趟自动生成 report.html；检测两个里程计", "A mapping/scripts/map_report（结论表、描黑图 + 轨迹、corr 曲线、里程计条数 / s）\nM mapping/scripts/save_mapping（存完自动生成报告）\nM mapping/scripts/scan_tf_bridge（/scan_link 加 odom_pubs，> 1 报红字）\nM CMakeLists.txt（install map_report）\nM D:\\ubuntu\\slam\\run_slam_windows.ps1（-Report；关窗口时给 autosave 生成报告；保存后自动打开报告；[监控] 两个里程计红字）"],
        ["11-04", "7756de5（合并 4adefee，已上车）", "删抗丢包；里程计只留快捷键 1，话题 /key1_odom", "D mapping/scripts/scan_relay\nM mapping/scripts/scan_tf_bridge（去掉 relay / 排队 / 去重 / 丢包统计，odom_topic 默认 /key1_odom；地面线首尾闭合轮廓修正）\nM mapping/launch/distributed_mapping_vehicle.launch.py（去 relay、录包；imu_encoder_mix remap 成 /key1_odom）\nM mapping/launch/distributed_mapping_pc*.launch.py（去 source、slam_mode，固定 async）\nM launch/move_test、move_new、final_start、save_path、odom_move_test.launch.py（remap /key1_odom，start_odom 默认 false）\nM scripts/record_seg.py（订 /key1_odom，不再自己起里程计）\nM mapping/scripts/map_report（去网络项；两个里程计按发布者个数 + 车动时 > 70 条/s；数据断只算中途断）\nM CMakeLists.txt（去 scan_relay）\nM D:\\ubuntu\\slam\\run_slam_windows.ps1（去 -Bag / -Rate / -Async、网络行；-Check 查 /key1_odom）"],
        ["11-04", "7756de5（合并 4adefee，已上车）", "高速建图：去畸变默认开、-Fast、EKF 融合（-Odom ekf）", "M mapping/scripts/scan_tf_bridge（deskew、stamp_at_end 默认 true；odom_check_topic、odom_rate 条数红字）\nM mapping/launch/distributed_mapping_pc_windows.launch.py（deskew 默认 true；odom_source key1 / ekf、imu_gyro_scale，起 ekf_input + ekf_node）\nM mapping/launch/distributed_mapping_pc.launch.py（deskew 默认 true）\nA mapping/scripts/ekf_input（车上时钟换算、填协方差、航向自检）\nA mapping/config/ekf.yaml（robot_localization：轮速 vx、vy + IMU vyaw）\nA mapping/config/mapper_fast.yaml（回环搜索 8 m）\nM CMakeLists.txt（install ekf_input）\nM D:\\ubuntu\\slam\\pixi.toml / pixi.lock（加 ros-humble-robot-localization；备份 .bak_20261104）\nM D:\\ubuntu\\slam\\run_slam_windows.ps1（-Odom、-GyroScale、-NoDeskew、-Fast；目录后缀 _fast / _ekf；[监控] 条数红字；-Check 查 /encoder、/IMU_data）"],
        ["11-04", "7756de5（合并 4adefee，已上车）", "赛道拟合：和规则 A/B/C/D 对齐并强制校正", "A mapping/scripts/fit_track（模板含黄线区、相似变换拟合、两段薄板样条校正、锥桶重画到线上 / 合并 / 赛道内清碎点 → map_track.*、track_fit.png / .json；FIT_TRACK 指定赛道）\nM mapping/scripts/save_mapping（描黑后自动调 fit_track）\nM mapping/scripts/map_report（结论表“赛道拟合”一行 + 叠加图；FIT_TRACK 变了重做）\nM CMakeLists.txt（install fit_track）\nM D:\\ubuntu\\slam\\run_slam_windows.ps1（-Track A|B|C|D）"],
        ["11-04", "7756de5（合并 4adefee，已上车）", "EKF 改用小车时间（不换算）", "M mapping/scripts/ekf_input（发 /ekf_clock，去掉时钟换算和 /ekf_odom_car）\nM mapping/launch/distributed_mapping_pc_windows.launch.py（ekf_node use_sim_time、/clock:=/ekf_clock；桥订 /ekf_odom）"],
        ["11-04", "7756de5（合并 4adefee，已上车）", "跑路径代码恢复原样，只改建图自己的里程计名；保存快捷键 a/b/c/d；去畸变按实测雷达频率", "R launch/move_test、move_new、final_start、save_path、odom_move_test.launch.py、scripts/record_seg.py 恢复 HEAD（照旧起 imu_encoder_mix、订 /encoder_imu_odom；改动备份 D:\\ubuntu\\slam\\bak_20261104_跑路径key1改动.patch）\nM mapping/launch/distributed_mapping_vehicle.launch.py（节点名 key1_imu_encoder_mix，话题 /key1_odom）\nM mapping/scripts/scan_tf_bridge（去畸变用实测 /scan 间隔中位数；[监控] 和 /scan_link 显示雷达实测 Hz）\nM 电脑 PowerShell 配置：a / b / c / d = 保存并按赛道 A / B / C / D 拟合；4 改成 -Mode filtered -Fast（原来是 -LiveSolid）\nM D:\\ubuntu\\slam\\run_slam_windows.ps1（保存时显示用哪种赛道）"],
        ["11-05", "63f1d6b（未上车）", "地面线过滤不再删挡板；两种投票地图（只投票 / 投票 + 地面剔除）实时看、保存、报告并排", "M mapping/scripts/scan_tf_bridge（GroundLineFilter 判据改成“车开过了线原来的位置”，删 ground_learn_move / ground_learn_turn，加 ground_cross_margin；另发不过滤的 /scan_pc_raw）\nA mapping/scripts/vote_map（/scan_pc_raw -> /map_vote_raw、/scan_pc -> /map_vote，按 slam_toolbox 位姿图节点的优化结果实时撤票重投）\nM mapping/scripts/save_mapping（存 map_vote_raw.*、map_vote.*）\nM mapping/scripts/map_report（“两种投票地图”并排 + 障碍格差多少）\nM mapping/launch/distributed_mapping_pc_windows.launch.py、distributed_mapping_pc.launch.py（vote 参数，默认 true）\nM mapping/rviz/slam_mapping.rviz（Map Vote Raw、Map Vote 两层，默认不打勾）\nM CMakeLists.txt（install vote_map）\nM D:\\ubuntu\\slam\\run_slam_windows.ps1（-NoVote）"],
        ["11-05", "未提交", "建图突跳：扫描先扣住等经 WiFi 晚到的里程计；锥桶实时描深", "M mapping/scripts/scan_tf_bridge（odom_wait，默认 1.0 s：扫描排队，等里程计时间戳追上再处理；[监控] 和 /scan_link 加扣住 / 等满 / 插值失败帧数）\nA mapping/scripts/cone_marks（/map 里认出的锥桶 -> /cone_marks 黑色实心圆盘，不改 /map）\nM mapping/scripts/map_report（结论表加“里程计晚到（WiFi）”一行）\nM mapping/launch/distributed_mapping_pc_windows.launch.py、distributed_mapping_pc.launch.py（odom_wait、cones 参数）\nM mapping/rviz/slam_mapping.rviz（Cone Marks 层，默认打勾）\nM CMakeLists.txt（install cone_marks）\nM D:\\ubuntu\\slam\\run_slam_windows.ps1（-NoCones）\nM 临时修改\\...\\模拟测试\\高速建图\\player.py（--odom-lag 模拟里程计成批晚到）、replay.ps1（-OdomLag、-OdomWait）"],
        ["11-10", "未提交", "按用户要求移除锥桶实时描深", "删除未跟踪的 mapping/scripts/cone_marks；移除 Windows / Linux launch 的 cones 启动项、RViz Cone Marks 层、CMake 安装项和电脑脚本 -NoCones；保存后 solidify_map 描黑保留。"],
        ["11-10", "未提交", "快捷键 2 改成轮速 + IMU 位姿的室外对照建图，4 保持原样", "M mapping/launch/distributed_mapping_pc_windows.launch.py（新增 odom_only：use_scan_matching=false、do_loop_closing=false）\nM D:\\ubuntu\\slam\\run_slam_windows.ps1（-OdomOnly、目录后缀 _odom_only、模式记录；补 UTF-8 BOM 保证 Windows PowerShell 5.1 可解析）\nM 电脑 PowerShell 配置：2 = -Mode filtered -Fast -OdomOnly，4 不变；2、4 单独运行\nM AGENTS.md、CLAUDE.md（同步快捷键约定）"],
    ], [0.9, 1.8, 3.4, 4.6]),
    ("note", "AB 路径拼接（save_ab_path.cpp、get_local_path.cpp 等）只在 临时修改\\AB路径\\ 里，还没进项目仓库（10-17 核对）。"),
])

MAPPING = dict(title="三、分布式建图", intro="", blocks=[
    ("table", ["状态", "文件", "说明"], [
        ["A", "mapping/scripts/scan_tf_bridge", "/scan → 720 点 /scan_pc，按扫描时间戳发 odom_pc→base_pc；收不到里程计报错。10-21：source auto/relay/direct；排队区按时间戳排序、去重、等里程计（relay 最多 3 s）、每秒最多约 80 帧；里程计缓存 120 s、跳变阈值随间隔放宽；转发 /odom_pc；每 5 s 发 /scan_link 网络统计。11-03 地面线过滤 GroundLineFilter：雷达系里不动、而车沿其法向走了 1 m（或转了 0.5 rad）的长直线认作地面线，删掉线上和线外的点（填 NaN）；ground_filter auto / fixed / off（未提交）。11-05 判据改成：雷达系里不动、且车开过了它之前所在位置的长直线（挡板开不过去）；另发不过滤的 /scan_pc_raw（63f1d6b）。11-05 odom_wait：扫描先扣住，等经 WiFi 晚到的里程计追上再处理，最多 1 s（未提交）"],
        ["D", "mapping/scripts/cone_marks", "11-10 用户要求删除 11-05 未提交的锥桶实时描深；启动项、RViz 图层、-NoCones 同时移除；保存后描黑保留"],
        ["A", "mapping/scripts/vote_map", "11-05 两种投票地图：/scan_pc_raw -> /map_vote_raw（只投票）、/scan_pc -> /map_vote（投票 + 地面剔除），投票规则同 slam_toolbox（经过 >= 3、命中 >= 20%），按位姿图节点优化结果局部刚体修正、撤票重投；save_mapping 存 map_vote*（63f1d6b）"],
        ["A", "mapping/scripts/map_report", "11-03 建图报告：读趟目录的地图和状态日志，写 report.html + report_map.png（结论：丢帧、corr 突跳、里程计个数、地面线、锥桶数）（未提交）"],
        ["A", "mapping/scripts/solidify_map", "11-03 存图后描黑：读 <趟目录>/map.raw（没有就读 map.pgm），调 obstacle_layer.solidify()，写 map_solid.* 和 solid.json 并登记进 complete.json（未提交）"],
        ["D", "mapping/scripts/scan_relay", "10-21 车端抗丢包转发（aeb9085 提交）；11-04 按用户要求删除（未提交）"],
        ["A", "mapping/scripts/ekf_input", "11-04 EKF 模式输入适配：/encoder、/IMU_data → /ekf/wheel、/ekf/imu（填协方差，车上时间不换算）+ /ekf_clock（车上时间当时钟给 ekf_node）；每 5 s 航向自检（未提交）"],
        ["A", "mapping/scripts/fit_track", "11-04 赛道拟合：描黑后的锥桶和规则 A/B/C/D 模板相似变换拟合，一律强制校正（两段薄板样条），锥桶重画到赛道线 / 黄线上、合并、赛道内清碎点 → map_track.*、track_fit.png / .json（未提交）"],
        ["A", "mapping/config/ekf.yaml、mapper_fast.yaml", "11-04 EKF 参数；高速两圈 SLAM 参数（回环搜索 8 m，其余同 mapper.yaml）（未提交）"],
        ["A", "mapping/scripts/map_autosave", "每 autosave_period 秒存到 <map_dir>/autosave/"],
        ["M", "mapping/scripts/mapping_status_logger", "known_cells、stalled 事件；10-11 加 odom/slam/corr；10-21 第 3、4 个参数指定扫描 / 里程计话题（电脑端用 /scan_pc、/odom_pc），记 /scan_link 为 link 字段（未提交）"],
        ["M", "mapping/scripts/save_mapping", "10-17 加 write_png，多存 map.png；10-19 有 /map_solid 时多存 map_solid.* 和 solid.json；yaml 的 image 名跟随文件名；11-03 没有 /map_solid 时存完自动调 solidify_map 描黑（失败不影响原图）（未提交）"],
        ["A", "mapping/scripts/obstacle_layer", "10-19 实体化障碍层第三版：在 /map 上只把赛道锥桶换成实心圆（solidify 函数，可离线跑）：边缘 = 房间外未知（outside_unknown）+ 长墙 + 贴外边的断墙，edge_margin 1.0 m 内不动；碎块合并后外环 0.08~0.25 m 内别的团 > 2 格不动；半径由近处单帧、凸向雷达、覆盖 >= 60° 的圆弧拟合；发 /map_solid（未提交）"],
        ["A", "mapping/launch/distributed_mapping_pc_windows.launch.py", "Windows 入口，pixi python 直接跑 scripts；10-21 source、slam_mode 参数；11-03 ground_filter / ground_rho / ground_theta_deg，solid 默认改 false；11-10 odom_only=true 时关闭激光扫描匹配和回环修正，位姿完全用 /key1_odom（未提交）"],
        ["M", "mapping/launch/distributed_mapping_pc.launch.py", "新增参数 autosave_period、beams、mapper_config；10-21 source（默认 auto）、slam_mode（默认 sync）；11-03 同 Windows 版（未提交）"],
        ["M", "mapping/launch/distributed_mapping_vehicle.launch.py", "只启动 imu_encoder_mix，不再启动 odom_tf_adapter；10-21 加 relay（scan_relay）、relay_buffer_s、record（ros2 bag record 到 bag_root）参数，默认都开（未提交）"],
        ["M", "mapping/config/mapper.yaml", "occupancy_threshold 0.35→0.20、min_pass_through 4→3；10-11 惩罚和回环参数；11-03 匹配 / 惩罚 / 回环改回默认（见下表）（未提交）"],
        ["A", "mapping/config/mapper_native.yaml", "不设两个阈值，其余同 mapper.yaml"],
        ["A", "scripts/Run_car_safe.sh", "安全版底盘驱动启动"],
        ["M", "CMakeLists.txt", "install(PROGRAMS) 加 map_autosave、view_path_on_map；10-19 obstacle_layer；10-21 scan_relay；11-03 solidify_map（未提交）"],
        ["A", "D:\\ubuntu\\slam\\run_slam_windows.ps1 / .bat", "建图 / -Check / -Save / -Mode / -AutoSave，自动限定网卡；10-21 -Bag（离线重建，本机域 77，放完自动保存，缺 metadata 自动 reindex）、-Rate、-Async，[监控] 网络一行，-Check 查 /scan_relay；11-03 -Ground auto|fixed|off、-GroundRho、-GroundDeg、-LiveSolid（实时描黑默认关），[监控] 地面线一行；11-10 -OdomOnly、_odom_only 目录后缀和模式记录，快捷键 2 使用（未提交）"],
    ], [0.7, 4.2, 5.3]),
    ("h2", "10-11 mapper 参数（两个 yaml 同改）"),
    ("table", ["参数", "原值", "新值"], [
        ["distance_variance_penalty", "（未设）", "1.0"],
        ["angle_variance_penalty", "（未设）", "1.8"],
        ["minimum_distance_penalty", "（未设）", "0.3"],
        ["minimum_angle_penalty", "（未设）", "0.6"],
        ["loop_search_maximum_distance", "4", "6"],
        ["loop_match_minimum_response_coarse / fine", "0.35 / 0.45", "0.30 / 0.40"],
    ], [4, 2, 2]),
    ("h2", "11-03 mapper 参数（两个 yaml 同改，操场锥桶等间距、空地无墙时被带偏）"),
    ("table", ["参数", "原值", "新值"], [
        ["correlation_search_space_dimension", "1.0（±0.5 m）", "0.5（±0.25 m，小于半个锥桶间距 1.3 m）"],
        ["coarse_search_angle_offset", "0.5（±28.6°）", "0.349（±20°）"],
        ["distance_variance_penalty / angle_variance_penalty", "1.0 / 1.8", "0.5 / 1.0"],
        ["minimum_distance_penalty / minimum_angle_penalty", "0.3 / 0.6", "0.5 / 0.9"],
        ["loop_search_maximum_distance", "6", "4"],
        ["loop_match_minimum_response_coarse / fine", "0.30 / 0.40", "0.35 / 0.45"],
    ], [4, 2, 2]),
])

OTHER = dict(title="四、路径、AB 与视觉", intro="", blocks=[
    ("h2", "CSV 路径与地图融合"),
    ("table", ["状态", "文件", "说明"], [
        ["A", "mapping/scripts/view_path_on_map", "地图 + CSV 发布节点"],
        ["A", "mapping/launch/view_path_on_map.launch.py、mapping/rviz/path_on_map.rviz", "Linux 入口和显示配置"],
        ["A", "D:\\ubuntu\\slam\\view_csv.ps1 / .bat", "Windows 入口"],
        ["A", "临时修改\\csv路径和slam融合\\拖拽CSV到这里.bat、drop_view.ps1", "拖拽入口"],
    ], [0.7, 5, 4.5]),
    ("h2", "AB 路径拼接（只在 临时修改\\AB路径\\，未进项目）"),
    ("table", ["状态", "文件", "说明"], [
        ["A", "src/save_ab_path.cpp、launch/save_ab_path.launch.py", "AB 轨迹录制"],
        ["M", "src/get_local_path.cpp", "7 处【AB改动】：读 CSV、镜像、拼接、去重、/ab_select"],
        ["M", "launch/move_test.launch.py", "ab 参数；get_local_path 加 ab_speed、laps、ab_fork_index"],
        ["M", "CMakeLists.txt", "注册 save_ab_path"],
    ], [0.7, 5, 4.5]),
    ("h2", "红绿灯视觉"),
    ("table", ["状态", "文件", "说明"], [
        ["A", "camera_capture/light_infer.py、light_bridge.py、light_listener.py、start_vision.sh、run.sh", "本地识别"],
        ["A", "camera_capture/record_video.py、check_env.sh、convert.sh、onnx2om.py、config.yaml、common/", "采集、检查、转换"],
        ["A", "camera_capture/remote_vision_bridge.py、remote_proto.py、start_remote_vision.sh", "远程识别小车端"],
        ["A", "camera_capture/pc_vision_client.py、start_pc_vision.ps1", "远程识别电脑端"],
        ["M（小车，未提交）", "camera_capture/light_bridge.py、start_vision.sh；A light_infer_daemon.py", "还没取回电脑"],
    ], [1.2, 5.5, 3.5]),
])

CHAPTERS = [INTRO, LOG, MAPPING, OTHER]
