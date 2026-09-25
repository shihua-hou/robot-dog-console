#!/bin/bash
# 安装并启用开机自启：sudo bash scripts/install_web_console_service.sh
set -e
WS=/home/linaro/robot_ws
UNIT=dog-web-console.service
SRC="$WS/scripts/$UNIT"
DST="/etc/systemd/system/$UNIT"

if [ "$(id -u)" -ne 0 ]; then
  echo "请用 root 安装： sudo bash $0"
  exit 1
fi

chmod +x "$WS/scripts/start_web_console.sh" "$WS/scripts/stop_web_console.sh"
cp "$SRC" "$DST"
systemctl daemon-reload
systemctl enable "$UNIT"
systemctl restart "$UNIT"
sleep 2
systemctl --no-pager --full status "$UNIT" || true
echo ""
echo "已启用开机自启：$UNIT"
echo "  立即启动: sudo systemctl start $UNIT"
echo "  查看状态: systemctl status $UNIT"
echo "  关闭自启: sudo systemctl disable $UNIT"
WLAN_IP=$(ip -4 -o addr show dev wlan0 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1)
echo "  浏览器:   http://${WLAN_IP:-<wlan-ip>}:8080/"
