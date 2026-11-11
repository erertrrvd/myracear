#!/bin/bash
# 在小车上跑红绿灯识别。
# 用法（小车上，root）：
#   bash run.sh            终端看状态（推荐先用）
#   bash run.sh show       弹窗（需要 MobaXterm X server，很慢）
#   bash run.sh video      录 30 秒带框视频到 /tmp/out.mp4，再 scp 回电脑看
#   bash run.sh images DIR 离线跑一个文件夹里的图片
# 其他参数可以接在后面，例如：  bash run.sh show --conf-box 0.4
set -u
cd "$(dirname "$(readlink -f "$0")")"
source /usr/local/Ascend/ascend-toolkit/set_env.sh

MODEL=output/yolov5n_bs1.om
[ -f "$MODEL" ] || { echo "找不到 $MODEL，先运行 convert.sh 并等它完成"; exit 1; }
if fuser /dev/video0 >/dev/null 2>&1; then
  echo "警告：/dev/video0 正在被别的程序占用："; fuser -v /dev/video0
fi

mode=${1:-term}; [ $# -gt 0 ] && shift
case "$mode" in
  term)   exec python3 light_infer.py --model "$MODEL" "$@" ;;
  show)   exec python3 light_infer.py --model "$MODEL" --show "$@" ;;
  video)  exec python3 light_infer.py --model "$MODEL" --save-video /tmp/out.mp4 --max-seconds 30 "$@" ;;
  images) d=${1:?用法：bash run.sh images 图片文件夹}; shift; exec python3 light_infer.py --model "$MODEL" --source "$d" "$@" ;;
  *) echo "用法：bash run.sh [term|show|video|images DIR]"; exit 1 ;;
esac
