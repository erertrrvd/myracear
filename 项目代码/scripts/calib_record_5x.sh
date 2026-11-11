#!/bin/bash
# ============================================================================
#  calib_record_5x.sh —— 初始点校准【验收测试】：
#  车停在同一位置不动，反复重启录制节点 N 次（默认 5 次），
#  每次把 cones_ref.csv 另存一份，最后汇总成表格。
#
#  验收标准（plan/初始点校准任务.md §7）：
#     同一位置重启 5 次，(dx,dy,dyaw) 的标准差 < 3cm 且 < 1°
#
#  用法：
#      bash /home/racecar/src/my_racecar/scripts/calib_record_5x.sh
#      RUNS=8 bash .../calib_record_5x.sh          # 想跑更多次
#
#  产出目录（固定，不再散落在 /home/racecar 下）：
#      /home/racecar/calib_logs/<时间戳>_acceptance/
#          run1.csv ~ runN.csv    每次检出的 4 个锥桶坐标
#          run1.log ~ runN.log    每次的节点完整输出
#      可用 CALIB_DIR=... 覆盖根目录
#
#  ⚠️ 安全：cone_calib_record 只订阅雷达 + 写文件，不发任何运动指令。
#  ⚠️ 全程不要按 Ctrl+C —— 本脚本会自动停掉每次的节点。
# ============================================================================
set +u

RUNS=${RUNS:-5}
# ★ 所有日志/产出统一写到这里，不要散落在 /home/racecar 下
CALIB_DIR=${CALIB_DIR:-/home/racecar/calib_logs}
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=${OUT:-$CALIB_DIR/${STAMP}_acceptance}
# ★ 参考基准也放仓库外，避免录制覆盖被 git 跟踪的文件、把工作区弄脏
CSV=${CSV:-$CALIB_DIR/cones_ref.csv}

if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
cd /home/racecar || exit 1
# shellcheck disable=SC1091
source install/setup.bash 2>/dev/null || true

mkdir -p "$OUT"

echo "=============================================================="
echo " 初始点校准 · 验收测试（同一位置重启 $RUNS 次）"
echo "=============================================================="
echo ""
echo "[检查] 雷达 /scan 是否有数据..."
timeout 3 ros2 topic hz /scan 2>&1 | head -4
echo "  （没显示 average rate 就是雷达没启动，先启雷达再跑本脚本）"
echo ""
echo "⚠️  重要：从第 1 次到第 $RUNS 次，"
echo "        【车不要动、锥桶也不要动】"
echo "    只有这样测出来的才是『感知重复性』。"
echo ""

for i in 5 4 3 2 1; do echo "  准备 $i ..."; sleep 1; done
echo ""
echo "开始，共 $RUNS 次..."
echo ""

OK=0
for i in $(seq 1 "$RUNS"); do
    LOG="$OUT/run${i}.log"
    echo "---------------- 第 $i / $RUNS 次 ----------------"

    timeout -s INT 15 ros2 run my_racecar cone_calib_record > "$LOG" 2>&1
    pkill -INT -f cone_calib_record 2>/dev/null || true
    sleep 1
    pkill -9 -f cone_calib_record 2>/dev/null || true

    if grep -q '已写入参考基准文件' "$LOG" 2>/dev/null; then
        cp -f "$CSV" "$OUT/run${i}.csv"
        OK=$((OK + 1))
        grep -E 'CONES_REF|new:|fp :' "$LOG" | sed 's/.*cone_calib_record\]: //'
    else
        echo "  !! 第 $i 次没有成功产出（累积帧数不够或没检测到）"
        echo "     请看 $LOG 的末尾找原因"
        tail -5 "$LOG"
    fi
    sleep 1
done

echo ""
echo "=============================================================="
echo " 汇总：成功 $OK / $RUNS 次"
echo "=============================================================="
printf "%-6s %-32s\n" "次数" "检出的 4 个锥桶 (x,y) 按距离升序"
for i in $(seq 1 "$RUNS"); do
    f="$OUT/run${i}.csv"
    if [ -f "$f" ]; then
        row=$(grep -v '^#' "$f" | grep -v '^x,y' | tr '\n' ' ')
        printf "%-6s %s\n" "#$i" "$row"
    else
        printf "%-6s (缺失)\n" "#$i"
    fi
done
echo ""
echo "产出目录: $OUT"
echo ""
echo "=============================================================="
echo " 请把上面【全部输出】复制发回给 AI，"
echo " 并用 MobaXterm 的 SFTP 把整个目录拷回电脑："
echo "   $OUT"
echo " （里面有 run1.csv ~ run${RUNS}.csv，AI 用它生成正式验收报告）"
echo "=============================================================="
