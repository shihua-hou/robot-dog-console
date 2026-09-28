#!/bin/bash
# 停止网页控制台相关进程（不影响 FAST-LIO / Nav2）
set +e
# 先停看门狗，避免立刻又拉起来
pkill -f 'scripts/web_console_watchdog.py' 2>/dev/null
sleep 0.5
pkill -f 'python3 -m http.server 8080' 2>/dev/null
ps -eo pid,args | awk '/python3 .*web_ui\/web_ops_node\.py/ && !/awk/ {print $1}' | while read -r p; do
  kill "$p" 2>/dev/null
done
pkill -f 'rosbridge_websocket' 2>/dev/null
pkill -f 'rosbridge_websocket_launch' 2>/dev/null
sleep 1
pkill -9 -f 'scripts/web_console_watchdog.py' 2>/dev/null
pkill -9 -f 'python3 -m http.server 8080' 2>/dev/null
ps -eo pid,args | awk '/python3 .*web_ui\/web_ops_node\.py/ && !/awk/ {print $1}' | while read -r p; do
  kill -9 "$p" 2>/dev/null
done
pkill -9 -f 'rosbridge_websocket' 2>/dev/null
echo "[web-console] stopped"
exit 0
