#!/bin/bash
# 网页控制台：静态页 :8080 + web_ops API :8090 + rosbridge :9090
# 不自动拉起建图/导航整链（需要时在网页里点「开始建图」或跑 start_ui_stack.sh）
set -e
WS=/home/linaro/robot_ws
export ROS_HOME=/tmp/ros_home
mkdir -p /tmp/ros_home/log "$WS" /tmp

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "[web-console] rosbridge :9090..."
if ! pgrep -f 'rosbridge_websocket' >/dev/null 2>&1; then
  nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    >/tmp/rosbridge.log 2>&1 &
  sleep 2
fi

echo "[web-console] web_ops :8090..."
if ! pgrep -f 'web_ui/web_ops_node.py' >/dev/null 2>&1; then
  nohup python3 "$WS/web_ui/web_ops_node.py" >/tmp/webops.log 2>&1 &
  sleep 1
fi

echo "[web-console] static HTTP :8080..."
if ! pgrep -f 'http.server 8080' >/dev/null 2>&1; then
  nohup python3 -m http.server 8080 --directory "$WS/web_ui" \
    >/tmp/web_http.log 2>&1 &
  sleep 0.5
fi

WLAN_IP=$(ip -4 -o addr show dev wlan0 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1)
UI_IP="${WLAN_IP:-$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -v '^127\.' | grep -v '^192\.168\.168\.' | head -1)}"
echo "[web-console] ready → http://${UI_IP:-<ip>}:8080/  (API :8090  rosbridge :9090)"
