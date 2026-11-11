#!/bin/bash
# 把 yolov5n.onnx 转成 output/yolov5n_bs1.om（后台运行，课程记录约 2 小时）。
# 用法（小车上，root，在 /home/racecar/light_infer 目录里）：  bash convert.sh
# 看进度：  tail -f convert.log     （Ctrl+C 只是停止查看，不会停止转换）
set -u
cd "$(dirname "$(readlink -f "$0")")"

[ -f yolov5n.onnx ] || { echo "找不到 yolov5n.onnx（文件名必须是 yolov5n.onnx）"; exit 1; }
[ -f /usr/local/Ascend/ascend-toolkit/set_env.sh ] || { echo "找不到 CANN 的 set_env.sh"; exit 1; }
source /usr/local/Ascend/ascend-toolkit/set_env.sh
command -v atc >/dev/null || { echo "找不到 atc，CANN 环境没加载成功"; exit 1; }

echo "空闲内存："; free -h | head -2
echo "ONNX 大小：$(du -h yolov5n.onnx | cut -f1)"
if pgrep -f onnx2om.py >/dev/null; then echo "已经有一个转换在跑了（onnx2om.py），不重复启动"; exit 1; fi

# bash 脚本带 Windows 换行会报错，保险起见处理一下（没有 dos2unix 就用 sed）
for f in common/onnx2om.sh common/util/atc.sh; do sed -i 's/\r$//' "$f"; done

mkdir -p output
nohup python3 onnx2om.py > convert.log 2>&1 &
echo "已在后台启动转换，进程号 $!"
echo "看进度：tail -f convert.log"
echo "看内存：watch -n 5 free -h"
echo "完成标志：ls -lh output/yolov5n_bs1.om"
