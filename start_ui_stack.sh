#!/bin/bash
# Bring up the full upper-computer stack: robot sensing + rosbridge + web API + static UI.
# Usage: bash /home/linaro/robot_ws/start_ui_stack.sh
set -e
WS=/home/linaro/robot_ws
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "[1/4] Robot sensors (Livox + SDK bridge；建图请在网页点「开始建图」)..."
bash "$WS/start_all.sh" sensors

echo "[2/4] Scan node for Nav2/AMCL (leveled height band)..."
if ! pgrep -f nav_scan_node >/dev/null; then
  bash "$WS/start_scan_node.sh"
fi

echo "[3/4] rosbridge WebSocket :9090..."
if ! pgrep -f rosbridge_websocket >/dev/null; then
  nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/rosbridge.log 2>&1 &
  echo "rosbridge pid=$!"
  sleep 2
fi

echo "[4/4] web_ops_node HTTP :8090 + static file server :8080..."
if ! pgrep -f web_ops_node >/dev/null; then
  nohup bash -c "source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && python3 $WS/web_ui/web_ops_node.py" \
    > /tmp/webops.log 2>&1 &
  echo "web_ops pid=$!"
fi
if ! pgrep -f "http.server 8080" >/dev/null; then
  nohup python3 -m http.server 8080 --directory "$WS/web_ui" > /tmp/web_http.log 2>&1 &
  echo "http.server pid=$!"
fi

# Prefer WLAN for browser access; eth is for dog SDK / Livox / switch LAN.
wlan_ip() {
  ip -4 -o addr show dev wlan0 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1
}
eth_ips() {
  ip -4 -o addr show dev eth0 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | tr '\n' ' '
}
WLAN_IP=$(wlan_ip)
ETH_IPS=$(eth_ips)
UI_IP="${WLAN_IP:-}"
if [ -z "$UI_IP" ]; then
  # fallback: first non-loopback that is not 192.168.168.x (dog link)
  UI_IP=$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -v '^127\.' | grep -v '^192\.168\.168\.' | head -1)
fi

echo ""
echo "UI stack ready."
echo "  Open (WLAN):  http://${UI_IP:-<wlan-ip>}:8080/"
echo "  rosbridge:    ws://${UI_IP:-<wlan-ip>}:9090"
echo "  API:          http://${UI_IP:-<wlan-ip>}:8090"
if [ -n "$WLAN_IP" ]; then
  echo "  wlan0:        $WLAN_IP"
fi
if [ -n "$ETH_IPS" ]; then
  echo "  eth0:         $ETH_IPS  (狗 SDK / Livox / 交换机，不要用这个开网页)"
fi
echo "  Dog eth:      192.168.168.168 (SDK)"
