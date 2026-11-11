#!/bin/bash
# Run_car.sh 的安全版：启动前清理上次残留，退出时把整组进程连根拔掉（不修改原 Run_car.sh）
PAT='hipnuc_imu/lib/hipnuc_imu/talker|lslidar_driver/lib/lslidar_driver/lslidar_driver_node|encoder/lib/encoder/encoder_node'

if pgrep -f "ros2 launch racecar Run_car.launch.py" >/dev/null; then
  echo "已有 Run_car 在运行：请先去原窗口 Ctrl+C，再启动。"
  exit 1
fi

LEFT=$(pgrep -f "$PAT")
if [ -n "$LEFT" ]; then
  echo "发现上次残留的驱动进程，先清理: $LEFT"
  kill $LEFT
  sleep 2
  LEFT=$(pgrep -f "$PAT")
  [ -n "$LEFT" ] && kill -9 $LEFT
fi

source /opt/ros/humble/setup.bash
source /home/racecar/install/local_setup.bash

set -m                                   # 后台任务各自独立进程组：pgid == $!
ros2 launch racecar Run_car.launch.py &
Run_car=$!

cleanup() {
  trap '' INT TERM HUP
  echo "终止所有后台进程..."
  ( sleep 15; kill -KILL -- -$Run_car 2>/dev/null ) &   # 看门狗：15 秒还没退干净就强杀整组
  WD=$!
  kill -INT $Run_car 2>/dev/null
  wait $Run_car 2>/dev/null
  kill -KILL -- -$Run_car 2>/dev/null                    # 扫尾：同组里被 launch 漏掉的残留
  kill $WD 2>/dev/null
  echo "所有进程已终止。"
  exit 0
}
trap cleanup INT TERM HUP

wait $Run_car
cleanup
