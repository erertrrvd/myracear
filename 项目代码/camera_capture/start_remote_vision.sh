#!/bin/bash
# 小车端远程视觉：采集摄像头 → 发给电脑识别 → 收回结果 → 发布 /traffic_state 等话题。
# 用法（小车上，root）：
#   bash start_remote_vision.sh                  默认监听 0.0.0.0:9000
#   BIND=192.168.5.100 bash start_remote_vision.sh   只监听车的 WiFi 地址（更安全）
#   bash start_remote_vision.sh --selftest       只测采集和编码的负载，不连电脑
#   bash start_remote_vision.sh --fps 10 --quality 70    额外参数传给 remote_vision_bridge.py
# 电脑上（Windows PowerShell）再运行 pc_vision_client.py，见 README。
set -u
cd "$(dirname "$(readlink -f "$0")")"

[ -f /opt/ros/humble/setup.bash ] || { echo "找不到 /opt/ros/humble/setup.bash"; exit 1; }
if fuser /dev/video0 >/dev/null 2>&1; then
  echo "摄像头 /dev/video0 被别的程序占用，先关掉它："; fuser -v /dev/video0; exit 1
fi
if pgrep -f remote_vision_bridge.py >/dev/null; then echo "remote_vision_bridge.py 已经在运行了"; exit 1; fi
if pgrep -f light_infer.py >/dev/null; then echo "light_infer.py（车上识别）正在运行，先停掉它再启动远程视觉"; exit 1; fi

BIND=${BIND:-0.0.0.0}
echo "本车地址：$(hostname -I)"
echo "电脑端运行：D:/light_ai/venv/Scripts/python.exe pc_vision_client.py --car <上面的车 IP>"
# 去掉 conda/CANN 留下的 PYTHONPATH，用 ROS 的 /usr/bin/python3（不需要 torch、NPU）
exec env -u PYTHONPATH bash -c 'source /opt/ros/humble/setup.bash && exec /usr/bin/python3 remote_vision_bridge.py "$@"' _ --bind "$BIND" "$@"
