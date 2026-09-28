#!/usr/bin/env python3
"""网页控制台常驻看门狗：保证 :8080 静态页 + :8090 web_ops + :9090 rosbridge 一直在。

- 每 HEARTBEAT_SEC 秒检查进程/端口，挂了就拉起
- 写心跳文件 /tmp/web_console_heartbeat（供排查）
- 作为 systemd Type=simple 主进程；挂掉由 systemd Restart=always 拉回
"""
from __future__ import annotations

import os
import signal
import socket
import subprocess
import sys
import time

WS = "/home/linaro/robot_ws"
ROS_SETUP = "source /opt/ros/humble/setup.bash; source /home/linaro/robot_ws/install/setup.bash"
ROS_HOME = "/tmp/ros_home"
HEARTBEAT_SEC = 5.0
HEARTBEAT_FILE = "/tmp/web_console_heartbeat"
LOG = "/tmp/web_console_watchdog.log"

# 避免短时间连环重启
_last_start: dict[str, float] = {}
MIN_RESTART_GAP = 8.0

_running = True


def log(msg: str) -> None:
    line = time.strftime("%Y-%m-%d %H:%M:%S") + " " + msg
    try:
        with open(LOG, "a") as f:
            f.write(line + "\n")
    except Exception:
        pass
    print(line, flush=True)


def cmdline(pid: int) -> str:
    try:
        return open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode()
    except Exception:
        return ""


def find_pids(*needles: str) -> list[int]:
    out = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        c = cmdline(int(name))
        if not c:
            continue
        if "extglob" in c or "web_console_watchdog" in c or "dump_bash_state" in c:
            continue
        if all(n in c for n in needles):
            out.append(int(name))
    return out


def port_open(port: int, host: str = "127.0.0.1") -> bool:
    try:
        with socket.create_connection((host, port), timeout=1.0):
            return True
    except OSError:
        return False


def bash_bg(cmd: str, log_path: str) -> None:
    env = dict(os.environ)
    env["ROS_HOME"] = ROS_HOME
    env["PYTHONUNBUFFERED"] = "1"
    full = f"{ROS_SETUP}; export ROS_HOME={ROS_HOME}; {cmd}"
    subprocess.Popen(
        ["bash", "-lc", full],
        stdout=open(log_path, "a"),
        stderr=subprocess.STDOUT,
        env=env,
        start_new_session=True,
    )


def can_start(key: str) -> bool:
    now = time.time()
    last = _last_start.get(key, 0.0)
    if now - last < MIN_RESTART_GAP:
        return False
    _last_start[key] = now
    return True


def ensure_http() -> bool:
    """静态页 :8080"""
    if port_open(8080) and find_pids("http.server", "8080"):
        return True
    if not can_start("http"):
        return False
    # 清掉僵死监听
    for p in find_pids("http.server", "8080"):
        try:
            os.kill(p, signal.SIGKILL)
        except Exception:
            pass
    time.sleep(0.3)
    log("restart static HTTP :8080")
    bash_bg(
        f"exec python3 -m http.server 8080 --directory {WS}/web_ui",
        "/tmp/web_http.log",
    )
    return False


def ensure_webops() -> bool:
    if port_open(8090) and find_pids("web_ui/web_ops_node.py"):
        return True
    if not can_start("webops"):
        return False
    for p in find_pids("web_ui/web_ops_node.py"):
        try:
            os.kill(p, signal.SIGKILL)
        except Exception:
            pass
    time.sleep(0.3)
    log("restart web_ops :8090")
    bash_bg(f"exec python3 {WS}/web_ui/web_ops_node.py", "/tmp/webops.log")
    return False


def ensure_rosbridge() -> bool:
    if port_open(9090) and find_pids("rosbridge_websocket"):
        return True
    if not can_start("rosbridge"):
        return False
    for p in find_pids("rosbridge_websocket"):
        try:
            os.kill(p, signal.SIGKILL)
        except Exception:
            pass
    # launch 父进程
    for p in find_pids("rosbridge_websocket_launch"):
        try:
            os.kill(p, signal.SIGKILL)
        except Exception:
            pass
    time.sleep(0.5)
    log("restart rosbridge :9090")
    # use_compression: 机器人现在走 WiFi 接网页端(实测到两个客户端 ping 40~375ms、
    # 抖动很大)，rosbridge 日志里持续刷 "Write queue full, dropping outgoing
    # message"——发送队列一直堆满在丢包，3D 点云等大部分消息送不到浏览器。
    # 开 permessage-deflate 压缩能显著减小 JSON 消息体积，缓解队列积压
    # （改善不了链路本身的延迟/抖动，但能让同样的带宽跑更多有效数据）。
    bash_bg(
        "exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml "
        "use_compression:=true",
        "/tmp/rosbridge.log",
    )
    return False


def write_heartbeat(ok_http: bool, ok_api: bool, ok_rb: bool) -> None:
    try:
        with open(HEARTBEAT_FILE, "w") as f:
            f.write(
                f"ts={time.time():.0f} http8080={int(ok_http)} "
                f"api8090={int(ok_api)} rosbridge9090={int(ok_rb)}\n"
            )
    except Exception:
        pass


def on_signal(signum, _frame):
    global _running
    log(f"signal {signum}, shutting down watchdog (children kept unless ExecStop)")
    _running = False


def main() -> int:
    os.makedirs(ROS_HOME + "/log", exist_ok=True)
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    log("web_console_watchdog start")

    # 首次立刻拉齐
    ensure_rosbridge()
    ensure_webops()
    ensure_http()
    time.sleep(2.0)

    while _running:
        ok_rb = ensure_rosbridge() or (port_open(9090) and bool(find_pids("rosbridge_websocket")))
        ok_api = ensure_webops() or (port_open(8090) and bool(find_pids("web_ui/web_ops_node.py")))
        ok_http = ensure_http() or (port_open(8080) and bool(find_pids("http.server", "8080")))
        # 再读一次真实端口
        ok_http, ok_api, ok_rb = port_open(8080), port_open(8090), port_open(9090)
        write_heartbeat(ok_http, ok_api, ok_rb)
        if not (ok_http and ok_api and ok_rb):
            log(f"health http={ok_http} api={ok_api} rosbridge={ok_rb}")
        # 可中断 sleep
        deadline = time.time() + HEARTBEAT_SEC
        while _running and time.time() < deadline:
            time.sleep(0.2)
    log("web_console_watchdog exit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
