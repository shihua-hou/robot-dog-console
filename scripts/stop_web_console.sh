#!/bin/bash
# 停止网页控制台相关进程（不影响 FAST-LIO / Nav2）
set +e
pkill -f 'python3 -m http.server 8080' 2>/dev/null
# 只杀 web_ops，避免误伤其它 python
ps -eo pid,args | awk '/python3 .*web_ui\/web_ops_node\.py/ && !/awk/ {print $1}' | while read -r p; do
  kill "$p" 2>/dev/null
done
pkill -f 'rosbridge_websocket' 2>/dev/null
sleep 1
# 仍活着则强杀
pkill -9 -f 'python3 -m http.server 8080' 2>/dev/null
ps -eo pid,args | awk '/python3 .*web_ui\/web_ops_node\.py/ && !/awk/ {print $1}' | while read -r p; do
  kill -9 "$p" 2>/dev/null
done
pkill -9 -f 'rosbridge_websocket' 2>/dev/null
echo "[web-console] stopped"
exit 0
