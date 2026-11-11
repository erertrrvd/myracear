#!/bin/bash
# ============================================================================
#  fix-time-on-boot.sh —— 开机自动修正系统时间（RTC 不可用）
# ============================================================================
#  背景：
#      小车主控的 RTC 硬件不可用：
#          $ hwclock --show
#          hwclock: ioctl(RTC_RD_TIME) to /dev/rtc0 to read the time failed:
#                   Invalid argument
#      导致每次开机系统时钟都回到 2023-11-22 附近。
#
#      而工作区里部分文件的 mtime 是 2026（在时钟曾为 2026 时写入），
#      于是 make 认为"源文件来自未来"，【每次 colcon build 都全量重编】
#      （21 个可执行文件，约 14 分钟）。
#
#  原理：
#      make 判断"要不要重编"的唯一依据是：
#          源文件 mtime  vs  对应 .o 的 mtime
#
#      所以只要保证【系统时钟 >= 所有文件 mtime】，增量编译就永远正常。
#
#      本脚本不"猜"该往后推多少，而是【直接读文件系统】：
#          取 max(git 提交时间, 工作区最新文件 mtime, 当前时钟)
#          然后如果当前时钟不够晚，就推进到该最大值 + 余量
#
#  为什么不用"每次重启 +1 天"：
#      那种做法推进的量只反映"重启了几次"，不反映"上次实际用到几点"。
#      若某次开机跑得久（跨过天边界），文件 mtime 会超过下次开机给的时钟，
#      漏洞再次出现。而且"重启次数"本身需要持久化 —— 又是个先有鸡还是先有蛋
#      的问题（存文件会被时钟问题影响，存 RTC 但 RTC 已坏）。
#
#  安装：
#      sudo bash scripts/install-fix-time.sh
#
#  验证：
#      date                                    # 看时间是否已被推进
#      systemctl status fix-time.service       # 看服务状态
#      journalctl -t fix-time -n 20            # 看日志
#      sudo systemctl start fix-time.service   # 手动触发一次
# ============================================================================

set -u

# ---------------------------- 可调参数 --------------------------------------
# 【为什么需要扫两个目录】
#
# 源文件和编译产物不在同一个目录：
#     /home/racecar/src/my_racecar/src/*.cpp        ← 你的代码
#     /home/racecar/build/my_racecar/CMakeFiles/**/*.o   ← 编译产物
#
# 而 make 判断"要不要重编"，比较的正是这两个：
#     源文件(src/)  vs  它对应的 .o(build/)
#
# 所以必须两个都扫：
#     只扫 src/  →  .o 可能比"算出的时钟"还晚 → 之后改代码会被跳过编译
#     只扫 build/ → 漏掉源头文件
#
# 【为什么 build/ 只扫 .o，不扫全部】
#     build/ 有几万个文件（.d、Makefile、CMakeCache 等），扫一遍要 1~3 秒
#     但只有 .o 参与 make 的"源 vs 目标"比较（只有 21 个）
#     → 只扫 .o，又快又准

# 源文件目录（全扫）
SCAN_SRC_DIRS="/home/racecar/src"

# 编译产物目录（只扫 .o）
SCAN_OBJ_DIRS="/home/racecar/build"

# git 仓库目录（用于读取最近提交时间）
REPO="/home/racecar/src/my_racecar"

# 时钟要排在"最新文件时间"之后多少秒（余量）
# 86400 = 1 天。留余量是为了应对"开机后立即产生新文件"的情况。
MARGIN=86400

# 日志标签（配合 journalctl -t 使用）
TAG="fix-time"
# ---------------------------------------------------------------------------

log() {
    echo "[$TAG] $*"
    logger -t "$TAG" "$*" 2>/dev/null || true
}

# 记录开始时间（用于统计扫描耗时）
START=$(date +%s%N 2>/dev/null || echo 0)

# ---- ① git 最近提交时间（= 开发机 PC 的时钟）----
GIT_T=0
if [ -d "$REPO/.git" ] && command -v git >/dev/null 2>&1; then
    GIT_T=$(cd "$REPO" && git log -1 --format=%ct 2>/dev/null || echo 0)
    [ -z "$GIT_T" ] && GIT_T=0
fi

# ---- ② 取"源文件"和"编译产物"里最新的 mtime ----
# 说明：
#   · 不用 `sort -rn`（几万文件时开销大），用 awk 单遍求最大值
#   · 源文件：src/ 下全部
#   · 编译产物：build/ 下只取 *.o（只有 21 个，极快；且只有它参与 make 比较）
max_mtime_of() {
    # $1 = 目录, $2 = 名称过滤（可选，如 '*.o'）
    local dir="$1" name="$2" m=0
    [ -d "$dir" ] || { echo 0; return; }
    if [ -n "$name" ]; then
        m=$(find "$dir" -type f -name "$name" -printf '%T@\n' 2>/dev/null \
            | awk 'BEGIN{x=0} {v=int($1); if(v>x) x=v} END{print x}')
    else
        m=$(find "$dir" -type f -printf '%T@\n' 2>/dev/null \
            | awk 'BEGIN{x=0} {v=int($1); if(v>x) x=v} END{print x}')
    fi
    [ -z "$m" ] && m=0
    echo "$m"
}

FILE_T=0
for d in $SCAN_SRC_DIRS; do
    t=$(max_mtime_of "$d" "")
    [ "$t" -gt "$FILE_T" ] && FILE_T=$t
done
for d in $SCAN_OBJ_DIRS; do
    t=$(max_mtime_of "$d" "*.o")
    [ "$t" -gt "$FILE_T" ] && FILE_T=$t
done

# ---- ③ 当前系统时钟 ----
SYS_T=$(date +%s)

# ---- ④ 取三者最大 ----
MAX=$GIT_T
[ "$FILE_T" -gt "$MAX" ] && MAX=$FILE_T
[ "$SYS_T"  -gt "$MAX" ] && MAX=$SYS_T

# ---- ⑤ 时钟不够晚就推进 ----
END=$(date +%s%N 2>/dev/null || echo 0)
if [ "$START" != "0" ] && [ "$END" != "0" ]; then
    ELAPSED_MS=$(( (END - START) / 1000000 ))
else
    ELAPSED_MS=-1
fi

if [ "$MAX" -eq 0 ]; then
    log "无法取得任何时间基准（无 git、无文件），跳过"
    exit 0
fi

if [ "$SYS_T" -lt "$MAX" ]; then
    TARGET=$((MAX + MARGIN))

    if date -s "@$TARGET" >/dev/null 2>&1; then
        log "时钟过早，已推进: $(date '+%Y-%m-%d %H:%M:%S')"
        log "  依据: git提交=$(date -d "@$GIT_T" '+%F %T' 2>/dev/null || echo -) " \
            "最新文件=$(date -d "@$FILE_T" '+%F %T' 2>/dev/null || echo -)"
        [ "$ELAPSED_MS" -ge 0 ] && log "  扫描耗时: ${ELAPSED_MS} ms"
    else
        log "设置时间失败（可能无权限），当前: $(date '+%Y-%m-%d %H:%M:%S')"
        exit 1
    fi
else
    log "时钟正常: $(date '+%Y-%m-%d %H:%M:%S')"
    [ "$ELAPSED_MS" -ge 0 ] && log "  扫描耗时: ${ELAPSED_MS} ms"
fi

exit 0
