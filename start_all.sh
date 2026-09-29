#!/bin/bash
# Idempotent bring-up.
#   bash start_all.sh                → 仅传感器 (Livox + SDK 桥)，不开建图
#   bash start_all.sh mapping        → 传感器 + Super-LIO + 预览（网页「开始建图」）
#   bash start_all.sh localization   → 传感器 + Super-LIO + TF 桥（导航用，无 map_building）
#   bash start_all.sh sensors        → 同上默认
#
# 全程持有 flock：ensure_one() 的"查进程数→不够就起"两步之间有空窗，两次
# 几乎同时的调用（网页双击、脚本和网页撞车、两个终端各跑一次）都会在空窗期
# 各自判定"没在跑"然后各启动一份，叠出重复的 super_lio_node/map_building_node
# 抢同一路雷达（2026-09-28 实测故障）。持锁串行化后，后到的调用会等前一个
# 跑完这整个脚本再执行，那时候 ensure_one() 看到的就是真实状态了。
exec 9>/tmp/start_all.sh.lock
flock 9

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
  # exec 9>&- 关掉继承来的锁 fd 再 exec 真正的长驻命令——不关的话 super_lio_node/
  # livox 这些一直跑到会话结束的后台进程会一直攥着 fd 9，这把 flock 就永远不会
  # 释放，后面任何一次 start_all.sh 调用都会在 flock 9 上死等（2026-09-28 实测：
  # 加 flock 当天午后连续 4 次"开始建图"全部卡死，就是这个）。
  nohup bash -c "exec 9>&-; $cmd" >"$log" 2>&1 &
  echo "  pid=$! → $log"
  sleep "$wait_s"
}

stop_mapping_nodes() {
  echo "[stop] mapping nodes (Super-LIO / map_building / lio_tf)"
  proc_kill 'msg_MID360s_launch' >/dev/null 2>&1 || true
  # do NOT kill livox here
  proc_kill 'super_lio Livox_mid360'
  proc_kill 'super_lio_node'
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
  # Super-LIO 与 relocation_node 互斥：两者都发 /lio/odom+/lio/cloud_world，
  # 同时跑会让建图页 3D 狗位姿在两路估计间跳变。
  if [ "$(proc_count relocation_node)" -gt 0 ]; then
    echo "[stop] relocation_node (conflicts with super_lio_node)"
    proc_kill 'dog_reloc.py'
    proc_kill 'relocation_node'
    sleep 0.5
  fi
  ensure_one 'super_lio_node' \
    'ros2 launch super_lio Livox_mid360.py rviz:=false' \
    /tmp/super_lio_run.log 5
  # 导航/建图均默认 dog 里程计：LIO 作 odom 易漂，拖垮 AMCL map→odom
  ensure_one 'lio_tf_bridge/lio_tf_bridge' \
    'ros2 launch lio_tf_bridge bridge.launch.py odom_source:=dog' \
    /tmp/lio_bridge.log 2
  if [ "$WANT_MAPPING" -eq 1 ]; then
  if [ "$(proc_count map_building_node)" -gt 1 ]; then
    echo "[fix] map_building pile-up=$(proc_count map_building_node), keep one"
    # 杀掉多余：保留最新 PID（proc_kill 全杀后再 ensure_one）
    proc_kill 'map_building.launch'
    proc_kill 'map_building_node'
    sleep 0.5
  fi
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
  if [ "$(proc_count super_lio_node)" -gt 0 ] || [ "$(proc_count map_building_node)" -gt 0 ]; then
    stop_mapping_nodes
  fi
fi

echo "ALL READY mode=$MODE livox=$(proc_count livox_ros_driver2_node) lio=$(proc_count super_lio_node)"
