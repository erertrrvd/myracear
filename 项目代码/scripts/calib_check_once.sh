#!/bin/bash
# ============================================================================
#  calib_check_once.sh —— 跑一次「初始点校准」（步骤2），自动收尾
# ============================================================================
#  用法：
#      bash /home/racecar/src/my_racecar/scripts/calib_check_once.sh
#
#  做什么：
#      1) 检查雷达 /scan
#      2) 5 秒倒计时（把车放好、退开）
#      3) 运行 cone_calib_run，读 cones_ref.csv 做指纹匹配 + 解位姿
#      4) 20 秒后自动停掉节点（不用按 Ctrl+C）
#      5) 把完整输出存到固定日志目录（不散落在 /home/racecar 下）：
#             /home/racecar/calib_logs/<时间戳>_calib_check.log
#         可用 CALIB_DIR=... 覆盖根目录
#
#  前提：cones_ref.csv 已经录好（先跑步骤1 / cone_calib_record）
#
#  ⚠️ 安全：cone_calib_run 只订阅雷达 + 读文件 + 打印，
#          不发布任何运动指令，不可能让车动。
# ============================================================================
set +u

# ★ 所有日志统一写到这里，不要散落在 /home/racecar 下
CALIB_DIR=${CALIB_DIR:-/home/racecar/calib_logs}
STAMP=$(date +%Y%m%d_%H%M%S)
mkdir -p "$CALIB_DIR"
LOG="$CALIB_DIR/${STAMP}_calib_check.log"

if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
cd /home/racecar || exit 1
# shellcheck disable=SC1091
source install/setup.bash 2>/dev/null || true

echo "=============================================================="
echo " 初始点校准（步骤 2）：测出本次停放相对标准停放的偏差"
echo "=============================================================="
echo ""
echo "[检查] 雷达 /scan 是否有数据..."
timeout 3 ros2 topic hz /scan 2>&1 | head -4
echo "  （没显示 average rate 就是雷达没启动）"
echo ""

REF=${REF:-$CALIB_DIR/cones_ref.csv}
if [ ! -f "$REF" ]; then
    echo "!! 找不到参考基准文件：$REF"
    echo "   请先跑步骤1：bash .../scripts/calib_record_5x.sh"
    exit 1
fi

echo "请确认："
echo "   1) 车已停在【起跑点】"
echo "   2) 接下来约 6 秒内不要碰车（节点只累积前 20 帧）"
echo ""
for i in 5 4 3 2 1; do echo "  $i ..."; sleep 1; done
echo ""
echo "开始校准..."
echo "=============================================================="

timeout -s INT 20 ros2 run my_racecar cone_calib_run 2>&1 | tee "$LOG"
pkill -INT -f cone_calib_run 2>/dev/null || true
sleep 1
pkill -9 -f cone_calib_run 2>/dev/null || true

echo ""
echo "=============================================================="
echo " 节点已停止。完整输出已保存到： $LOG"
echo ""
echo " 请把上面【从 步骤 2 那一行到最后】的全部内容复制发回给 AI，"
echo " 重点是这段："
echo "   CALIB ok=... dx=... dy=... dyaw=... err=... lambda=... maxres=..."
echo "=============================================================="
