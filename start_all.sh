#!/bin/bash
# Idempotent bring-up.
#   bash start_all.sh                → 仅传感器 (Livox + SDK 桥)，不开建图
#   bash start_all.sh mapping        → 传感器 + FAST-LIO + 预览（网页「开始建图」）
#   bash start_all.sh localization   → 传感器 + FAST-LIO + TF 桥（导航用，无 map_building）
#   bash start_all.sh sensors        → 同上默认
source /opt/ros/humble/setup.bash
source /home/linaro/robot_ws/install/setup.bash

MODE="${1:-sensors}"
case "$MODE" in
  mapping|all|slam) WANT_MAPPING=1; WANT_LIO=1 ;;
  localization|nav|localize) WANT_MAPPING=0; WANT_LIO=1 ;;
  *) WANT_MAPPING=0; WANT_LIO=0 ;;
esac

proc_count() {
  python3 -c "
import os
needle='$1'
n=0
for pid in os.listdir('/proc'):
  if not pid.isdigit(): continue
  try:
    c=open(f'/proc/{pid}/cmdline','rb').read().replace(b'\\0',b' ').decode()
  except Exception:
    continue
  if needle in c and 'extglob' not in c and 'start_all.sh' not in c and 'web_ops' not in c:
    n+=1
print(n)
"
}

proc_kill() {
  python3 -c "
import os, signal, time
needle='$1'
me, parent = os.getpid(), os.getppid()
for sig in (signal.SIGTERM, signal.SIGKILL):
  for pid in os.listdir('/proc'):
    if not pid.isdigit(): continue
    p=int(pid)
    if p in (me, parent): continue
    try:
      c=open(f'/proc/{p}/cmdline','rb').read().replace(b'\\0',b' ').decode()
    except Exception:
      continue
    if needle in c and 'extglob' not in c and 'start_all.sh' not in c and 'web_ops' not in c:
      try: os.kill(p, sig)
      except Exception: pass
  time.sleep(0.7)
"
}

ensure_one() {
  local needle="$1" cmd="$2" log="$3" wait_s="${4:-2}"
  local n
  n=$(proc_count "$needle")
  if [ "$n" -eq 1 ]; then
    echo "[ok] 1x $needle"
    return 0
  fi
  if [ "$n" -gt 1 ]; then
    echo "[fix] ${n}x $needle — collapse to one"
    proc_kill "$needle"
  fi
  echo "[start] $cmd"
  nohup bash -c "$cmd" >"$log" 2>&1 &
  echo "  pid=$! → $log"
  sleep "$wait_s"
}

stop_mapping_nodes() {
  echo "[stop] mapping nodes (FAST-LIO / map_building / lio_tf)"
  proc_kill 'msg_MID360s_launch' >/dev/null 2>&1 || true
  # do NOT kill livox here
  proc_kill 'fast_lio mapping.launch'
  proc_kill 'fastlio_mapping'
  proc_kill 'map_building.launch'
  proc_kill 'map_building_node'
  # lio_tf only needed with LIO; stop when not mapping
  proc_kill 'lio_tf_bridge bridge.launch'
  proc_kill 'lio_tf_bridge/lio_tf_bridge'
}

cp /home/linaro/robot_ws/src/livox_ros_driver2/config/MID360s_config.json \
   /home/linaro/robot_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/ 2>/dev/null || true

LIVOX_N=$(proc_count 'livox_ros_driver2_node')
LAUNCH_N=$(proc_count 'msg_MID360s_launch')
if [ "$LIVOX_N" -gt 1 ] || [ "$LAUNCH_N" -gt 1 ]; then
  echo "[fix] Livox pile-up nodes=$LIVOX_N launches=$LAUNCH_N"
  proc_kill 'msg_MID360s_launch'
  proc_kill 'livox_ros_driver2_node'
fi

ensure_one 'livox_ros_driver2_node' \
  'ros2 launch livox_ros_driver2 msg_MID360s_launch.py' \
  /tmp/livox_run.log 6

ensure_one 'genisom_bridge/genisom_bridge' \
  'ros2 launch genisom_bridge bridge.launch.py' \
  /tmp/bridge_run.log 3

if [ "$WANT_LIO" -eq 1 ]; then
  ensure_one 'fastlio_mapping' \
    'ros2 launch fast_lio mapping.launch.py rviz:=false' \
    /tmp/fastlio_run.log 5
  ensure_one 'lio_tf_bridge/lio_tf_bridge' \
    'ros2 launch lio_tf_bridge bridge.launch.py' \
    /tmp/lio_bridge.log 2
  if [ "$WANT_MAPPING" -eq 1 ]; then
    ensure_one 'map_building_node' \
      'ros2 launch nav2_tools map_building.launch.py' \
      /tmp/map_building.log 1
  else
    # 导航定位模式：不要挂着建图预览节点抢 CPU
    if [ "$(proc_count map_building_node)" -gt 0 ]; then
      proc_kill 'map_building.launch'
      proc_kill 'map_building_node'
    fi
  fi
else
  # 默认模式：确保建图没在后台偷跑
  if [ "$(proc_count fastlio_mapping)" -gt 0 ] || [ "$(proc_count map_building_node)" -gt 0 ]; then
    stop_mapping_nodes
  fi
fi

echo "ALL READY mode=$MODE livox=$(proc_count livox_ros_driver2_node) lio=$(proc_count fastlio_mapping)"
