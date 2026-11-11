#!/bin/bash
# 红绿灯视觉一键启动：先把识别守护进程拉起来加载模型，再跑话题节点。
#
# 为什么要分成两个：
#   light_infer_daemon.py  加载模型后进入 socket 待命。跑车全程不推理、
#                          不读摄像头，CPU ≈ 0；收到 on 才真正开始认灯。
#   light_bridge.py        ROS 话题节点，把控制节点的 /traffic_light_req
#                          翻译成 on / off 命令发给守护进程。
#
# 为什么要在发车前启动：这块板子上加载模型要 13~15 秒（import torch 就占 11 秒）。
# 发车前你还在摆车、按 s，这段时间正好把模型加载掉，赛道上就不用等了。
# 到停车点发 on 之后，约 100ms 就能出第一帧识别结果。
#
# 用法（小车上，root，在本目录里）：
#   bash start_vision.sh
#   bash start_vision.sh --conf-box 0.4 --conf-lamp 0.4    # 透传给守护进程
# Ctrl+C 停 bridge，守护进程会被一起带走。
set -u

cd "$(dirname "$(readlink -f "$0")")"

MODEL=output/yolov5n_bs1.om
STATE=/tmp/light_state.txt
SOCK=/tmp/light_infer.sock
DAEMON_LOG=infer.log
ROS_PY=/usr/bin/python3
# 守护进程要用装了 torch / ais_bench 的 conda python，不是 ROS 那个
INFER_PY=${INFER_PY:-/usr/local/miniconda3/bin/python3}

[ -f "$MODEL" ] || { echo "找不到 $MODEL，先运行 bash convert.sh 并等它完成"; exit 1; }
[ -x "$INFER_PY" ] || { echo "找不到跑识别的 python3：$INFER_PY"; exit 1; }
[ -f /opt/ros/humble/setup.bash ] || { echo "找不到 /opt/ros/humble/setup.bash"; exit 1; }

# ---------------------------------------------------------------- 清理上次的残留
if pgrep -f light_bridge.py >/dev/null; then
  echo "light_bridge.py 已经在运行了"; exit 1
fi
if pgrep -f light_infer_daemon.py >/dev/null; then
  echo "发现残留的识别守护进程，先清掉"
  pkill -f light_infer_daemon.py; sleep 1
fi
rm -f "$SOCK"

# ---------------------------------------------------------------- 1) 识别守护进程（后台，先加载）
CANN_ENV=/usr/local/Ascend/ascend-toolkit/set_env.sh
: > "$DAEMON_LOG"
(
  # CANNBASE 先清掉，否则 set_env.sh 会走「重复 source」的分支
  CANNBASE=
  [ -f "$CANN_ENV" ] && source "$CANN_ENV"
  exec "$INFER_PY" light_infer_daemon.py \
       --model "$MODEL" --socket "$SOCK" --state-file "$STATE" "$@"
) >>"$DAEMON_LOG" 2>&1 &
DAEMON_PID=$!

cleanup() {
  kill "$DAEMON_PID" 2>/dev/null
  wait "$DAEMON_PID" 2>/dev/null
  pkill -f light_infer_daemon.py 2>/dev/null
  rm -f "$SOCK"
  echo "已停止话题节点和识别守护进程"
}
trap cleanup EXIT

# ---------------------------------------------------------------- 2) 等守护进程就绪
echo "识别守护进程已在后台启动（PID $DAEMON_PID），正在加载模型..."
echo "加载期间不读摄像头、不推理；加载完进入待命，CPU ≈ 0"
echo "日志：$DAEMON_LOG"
READY=0
for i in $(seq 1 90); do
  kill -0 "$DAEMON_PID" 2>/dev/null || {
    echo "守护进程退出了，$DAEMON_LOG 最后几行："; tail -20 "$DAEMON_LOG"; exit 1
  }
  if grep -q "守护进程就绪" "$DAEMON_LOG" 2>/dev/null; then READY=1; break; fi
  sleep 1
done
if [ "$READY" != 1 ]; then
  echo "等了 90 秒守护进程还没就绪，最后几行日志："; tail -20 "$DAEMON_LOG"; exit 1
fi
echo "识别守护进程就绪（待命中，不占 CPU）"

# ---------------------------------------------------------------- 3) 话题节点（前台）
echo "启动话题节点（到停车点才会让守护进程开始认灯）"
# 去掉 CANN 的 PYTHONPATH：bridge 用 ROS 的 python3 跑，混进 CANN 的路径会崩
if env -u PYTHONPATH bash -c "source /opt/ros/humble/setup.bash && exec $ROS_PY light_bridge.py --socket $SOCK --state-file $STATE"; then
  echo "已停止话题节点"
else
  rc=$?                       # 必须紧跟在后面取，迟一行就是别的命令的返回码了
  if [ "$rc" -eq 130 ]; then
    echo "已停止话题节点（Ctrl+C）"
  else
    echo "light_bridge.py 异常退出，返回码 $rc"
  fi
fi
exit 0
