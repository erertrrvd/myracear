# 进度记录

## 2026-10-17 建图快捷键
- 小车输入 `1` = 启动里程计（直接 `ros2 launch distributed_mapping_vehicle.launch.py`，按用户要求不做“已在运行”判断）；写入小车 `~/.bashrc` 的命令块在备份文档 `windows建图linux建图分布式通信\README.md`「快捷键」一节，需用户在小车执行一次。
- 电脑 PowerShell 输入 `2` = `run_slam_windows.bat -Mode filtered`：PowerShell 里 `2` 是数字字面量，不能做别名或函数，改用 `$PROFILE` 里 PSReadLine 的 Enter 处理器替换整行。
- 电脑 `3` = `run_slam_windows.bat -Save`。`save_mapping` 新增 `map.png`（标准库 zlib 写灰度 PNG，Windows 可直接看），未提交。
- 10-17 20:08 那次建图（run_20261017_2008_filtered）odom/slam 5 分钟一直是 (0,0)，日志报 stalled，图只是原地一圈扫描：车没动或里程计没在更新，待用户确认。
- 建图漂移仍在等新参数下的 `.run1-live.jsonl`；mapper 两个 yaml 和 `mapping_status_logger` 仍未提交。

## 2026-10-11 红绿灯视觉（本地/远程）与小车 git 同步
- 远程视觉（小车传图、电脑识别）在真车上跑通：往返 p50 约 70–110 ms；偶发 WiFi 抖动（p95 约 3.4 秒）导致 `stale` 闪烁，原因未查清。
- 小车 `main` 和电脑 `main` 都是 `641ae59`（经小车分支 `from-pc` 快进合并）。小车提交 `02a512a` 带来运动侧的 `light_mode`、`/traffic_light_req`；`remote_vision_bridge.py` 新增默认关闭的 `--enable-control`（只验证过语法，没在车上跑）。
- 未完成：小车上未提交的 `light_bridge.py`、`start_vision.sh`、`light_infer_daemon.py` 还没取回电脑（补丁在小车 `/tmp/`，用户的 `scp` 误在小车终端里运行，需重做）；读完再判断和 `--enable-control` 的兼容。
- 文档：`事项\临时修改\yolov5n电脑分布式通信和本地\`（本地/远程指令集）、`git和小车修改冲突解决方法\指令集.md`。
- 详细交接：`C:\Users\lzc2\.claude\handovers\20261011-1113-红绿灯远程视觉同步.md`

## 2026-10-11 建图漂移
- 第二圈、高速外围漂移。已改（未提交、未实车测试）：`mapping/config/mapper.yaml` 与 `mapper_native.yaml` 放宽扫描匹配惩罚、放宽回环；`mapping/scripts/mapping_status_logger` 增加 odom/slam/corr 位姿记录。
- 小车 car-odom.service 已要求撤销，改回手动：Run_car → 等约 10 秒 → `ros2 launch my_racecar distributed_mapping_vehicle.launch.py`。
- 待办：拿新日志判断漂移来源；方案（慢速建图、录包离线建图、去畸变、雷达提频、赛道几何配准）等用户选择。
- 详细交接：`C:\Users\lzc2\.claude\handovers\20261011-1110-建图漂移.md`
