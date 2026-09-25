#!/bin/bash
# Start Nav2 with selected map (no duplicate standalone map_server).
# Usage: bash /home/linaro/robot_ws/nav2_start.sh [map.yaml]
source /opt/ros/humble/setup.bash
source /home/linaro/robot_ws/install/setup.bash

MAP="${1:-/home/linaro/robot_ws/maps/map.yaml}"
PARAMS="/home/linaro/robot_ws/src/nav2_tools/nav2_params.yaml"

# Ensure leveled /scan for AMCL / costmaps (nav_scan_node, not stock laserscan)
if ! pgrep -f nav_scan_node >/dev/null; then
  bash /home/linaro/robot_ws/start_scan_node.sh
  sleep 2
fi

# Stop previous Nav2
pkill -f component_container_isolated 2>/dev/null
pkill -f 'nav2_bringup' 2>/dev/null
sleep 2

# Ensure genisom + lio_tf for cmd_vel and odom
if ! pgrep -f genisom_bridge >/dev/null; then
  nohup ros2 launch genisom_bridge bridge.launch.py > /tmp/bridge_run.log 2>&1 &
  sleep 4
fi
if ! pgrep -f lio_tf_bridge >/dev/null; then
  nohup ros2 launch lio_tf_bridge bridge.launch.py > /tmp/lio_bridge.log 2>&1 &
  sleep 3
fi

nohup ros2 launch nav2_bringup bringup_launch.py \
  params_file:="$PARAMS" \
  map:="$MAP" \
  use_sim_time:=False \
  autostart:=True \
  > /tmp/nav2.log 2>&1 &
echo "NAV2_PID=$!"
sleep 12
echo ====NAV2_LOG====
grep -iE "Creating bond|Map received|Initialized|error|Exception" /tmp/nav2.log | tail -12
