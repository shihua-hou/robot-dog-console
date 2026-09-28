#!/bin/bash
# 一次性拉起网页三件套（看门狗也会做同样的事；本脚本便于手动）
set -u
WS=/home/linaro/robot_ws
export ROS_HOME=/tmp/ros_home
mkdir -p /tmp/ros_home/log

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# 若看门狗已在跑，只做一次 ensure；否则直接起看门狗（前台由 systemd 管）
if ! pgrep -f 'scripts/web_console_watchdog.py' >/dev/null 2>&1; then
  # 手动模式：后台起看门狗
  nohup python3 "$WS/scripts/web_console_watchdog.py" >/tmp/web_console_watchdog.log 2>&1 &
  echo "[web-console] watchdog pid=$!"
  sleep 3
else
  echo "[web-console] watchdog already running"
fi

WLAN_IP=$(ip -4 -o addr show dev wlan0 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1)
UI_IP="${WLAN_IP:-$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -v '^127\.' | grep -v '^192\.168\.168\.' | head -1)}"
echo "[web-console] → http://${UI_IP:-<ip>}:8080/  (API :8090  rosbridge :9090)"
# 健康摘要
for p in 8080 8090 9090; do
  if timeout 1 bash -c "echo >/dev/tcp/127.0.0.1/$p" 2>/dev/null; then
    echo "  :$p OK"
  else
    echo "  :$p DOWN"
  fi
done
