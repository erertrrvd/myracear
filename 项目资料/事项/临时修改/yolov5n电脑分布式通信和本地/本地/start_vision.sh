#!/bin/bash
# 一键启动：后台跑识别（light_infer.py，root 的 conda python3 + 昇腾），前台跑 ROS 话题发布（light_bridge.py，ROS 的 python）。
# Ctrl+C 两个一起停。
# 用法（小车上，root，在本目录里）：
#   bash start_vision.sh                     默认参数
#   bash start_vision.sh --conf-box 0.4      额外参数传给 light_infer.py
# 识别的日志在 infer.log
set -u
cd "$(dirname "$(readlink -f "$0")")"
MODEL=output/yolov5n_bs1.om
STATE=/tmp/light_state.txt

[ -f "$MODEL" ] || { echo "找不到 $MODEL，先运行 bash convert.sh 并等它完成"; exit 1; }
[ -f /opt/ros/humble/setup.bash ] || { echo "找不到 /opt/ros/humble/setup.bash"; exit 1; }
if fuser /dev/video0 >/dev/null 2>&1; then
  echo "摄像头 /dev/video0 被别的程序占用，先关掉它："; fuser -v /dev/video0; exit 1
fi
if pgrep -f "light_infer.py" >/dev/null; then echo "light_infer.py 已经在运行了"; exit 1; fi

# 1) 识别：用 CANN 环境 + root 的 python3（conda，装了 ais_bench）
(
  source /usr/local/Ascend/ascend-toolkit/set_env.sh
  exec python3 light_infer.py --model "$MODEL" --state-file "$STATE" "$@"
) > infer.log 2>&1 &
INFER=$!
cleanup() { kill "$INFER" 2>/dev/null; wait "$INFER" 2>/dev/null; echo "已停止识别和话题发布"; }
trap cleanup EXIT

echo "识别已在后台启动（进程 $INFER），等待第一帧结果..."
for _ in $(seq 1 60); do
  kill -0 "$INFER" 2>/dev/null || { echo "识别进程退出了，infer.log 最后几行："; tail -20 infer.log; exit 1; }
  [ -f "$STATE" ] && [ $(( $(date +%s) - $(stat -c %Y "$STATE") )) -le 2 ] && break
  sleep 1
done
echo "识别在运行：$(cat "$STATE" 2>/dev/null)"

# 2) 话题发布：去掉 CANN 的 PYTHONPATH，用 ROS 环境 + /usr/bin/python3
env -u PYTHONPATH bash -c 'source /opt/ros/humble/setup.bash && exec /usr/bin/python3 light_bridge.py'
