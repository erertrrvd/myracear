#!/bin/bash
# ============================================================================
#  install-fix-time.sh —— 一键安装"开机自动修正时钟"服务
# ============================================================================
#  用法：
#      sudo bash scripts/install-fix-time.sh
#
#  它会做：
#      ① 把 fix-time-on-boot.sh 复制到 /usr/local/bin/
#      ② 把 fix-time.service 复制到 /etc/systemd/system/
#      ③ systemctl enable + start
#      ④ 立刻执行一次并打印结果
#
#  卸载：
#      sudo bash scripts/install-fix-time.sh uninstall
# ============================================================================

set -e

SCRIPT_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/fix-time-on-boot.sh"
SERVICE_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/fix-time.service"

SCRIPT_DST="/usr/local/bin/fix-time-on-boot.sh"
SERVICE_DST="/etc/systemd/system/fix-time.service"

# ---- 卸载分支 ----
if [ "${1:-}" = "uninstall" ]; then
    echo "== 卸载 fix-time 服务 =="
    systemctl stop    fix-time.service 2>/dev/null || true
    systemctl disable fix-time.service 2>/dev/null || true
    rm -f "$SERVICE_DST" "$SCRIPT_DST"
    systemctl daemon-reload
    echo "已卸载。"
    exit 0
fi

# ---- 权限检查 ----
if [ "$(id -u)" -ne 0 ]; then
    echo "错误：需要 root 权限。请用： sudo bash $0"
    exit 1
fi

if [ ! -f "$SCRIPT_SRC" ]; then
    echo "错误：找不到 $SCRIPT_SRC"
    exit 1
fi
if [ ! -f "$SERVICE_SRC" ]; then
    echo "错误：找不到 $SERVICE_SRC"
    exit 1
fi

echo "== 安装 fix-time 服务 =="
echo

# ---- ① 安装主脚本 ----
echo "[1/4] 安装脚本 -> $SCRIPT_DST"
install -m 0755 "$SCRIPT_SRC" "$SCRIPT_DST"

# ---- ② 安装 systemd unit ----
echo "[2/4] 安装服务 -> $SERVICE_DST"
install -m 0644 "$SERVICE_SRC" "$SERVICE_DST"

# ---- ③ 启用 + 启动 ----
echo "[3/4] 启用并启动服务"
systemctl daemon-reload
systemctl enable fix-time.service
systemctl start  fix-time.service

echo
echo "[4/4] 服务状态："
systemctl status fix-time.service --no-pager -l | head -15 || true

echo
echo "== 当前系统时间 =="
date

echo
echo "== 最近日志 =="
journalctl -t fix-time -n 10 --no-pager 2>/dev/null || echo "(无日志)"

echo
echo "✅ 安装完成。"
echo
echo "接下来建议："
echo "  ① 编译一次（全量，让 .o 时间戳跟上）"
echo "       cd /home/racecar && time colcon build --packages-select my_racecar"
echo "  ② 再编一次验证增量（应该只要几秒）"
echo "       time colcon build --packages-select my_racecar"
echo "  ③ 重启验证（最关键）"
echo "       sudo reboot"
echo "       date"
echo "       cd /home/racecar && time colcon build --packages-select my_racecar"
