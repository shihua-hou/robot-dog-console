#!/bin/bash
# /cloud_registered → /scan: level with the calibrated gravity vector
# (config/lidar_extrinsic.yaml), slice z=[0.10,1.0], publish base_footprint.
source /opt/ros/humble/setup.bash
source /home/linaro/robot_ws/install/setup.bash
pkill -f pointcloud_to_laserscan_node 2>/dev/null || true
pkill -f nav_scan_node.py 2>/dev/null || true
pkill -f 'nav2_tools.nav_scan_node' 2>/dev/null || true
sleep 0.3
nohup python3 -u /home/linaro/robot_ws/src/nav2_tools/nav2_tools/nav_scan_node.py \
  --ros-args \
  -p z_min:=0.10 -p z_max:=1.0 \
  -p range_min:=0.20 -p range_max:=8.0 \
  -p angle_increment:=0.01 \
  > /tmp/scan_node.log 2>&1 &
echo "nav_scan_node pid=$!"
