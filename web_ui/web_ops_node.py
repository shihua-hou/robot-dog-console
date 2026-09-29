#!/usr/bin/env python3
"""web_ops_node - 网页上位机后端代理节点（v3：ROS + 管理 HTTP API）

ROS 订阅(前端 -> 主控):
  /web/nav_cmd     geometry_msgs/Twist   linear.x/y=目标 angular.z=yaw(rad) -> Nav2
  /web/nav_cancel  std_msgs/Empty        取消当前导航目标
  /web/init_pose   geometry_msgs/Twist   linear.x=x linear.y=y angular.z=yaw(rad) -> /initialpose
  /web/start_nav2  std_msgs/Empty        启动 Nav2（默认 map.yaml）
  /web/stop_nav2   std_msgs/Empty        停止 Nav2
  /web/mapping     std_msgs/Empty        确保 Super-LIO 建图链运行

ROS 发布(主控 -> 前端):
  /web/nav_status  std_msgs/String  idle|navigating|reached|aborted|canceled[:detail]
  /web/sys_status  std_msgs/String  JSON services + nav
  /web/robot_pose  geometry_msgs/PoseStamped  map→base_footprint（网页画激光/机身用）

HTTP API (端口 8090, CORS 开放):
  GET  /api/maps
  GET  /api/map/pgm?name=x.pgm
  GET  /api/map/preview?name=x.pgm   -> PNG 缩略图（浏览器可直接显示）
  POST /api/map/edit
  POST /api/map/delete
  POST /api/mapping/save     -> {name} 保存建图（可先 stop 再 save，或运行中直接 save）
  POST /api/mapping/stop     -> 停止建图并落盘 PCD（不转换）
  POST /api/mapping/discard  -> 停止建图并丢弃（不写地图）
  POST /api/nav2/start       -> {map} 启动定位栈 + 单例 Nav2 + auto_relocalize
  POST /api/nav2/stop
  POST /api/nav2/relocalize  -> 手动触发激光匹配重定位
  POST /api/nav2/reload_map  -> {map, reloc?} 热换图并可选强制重定位
  GET  /api/sysinfo
  GET  /api/log?file=xxx&n=50
  GET  /api/video/mjpeg      -> RTSP→MJPEG 代理（浏览器可直接 <img>）
  GET  /api/video/snapshot   -> 单帧 JPEG
  POST /api/restart_rosbridge
  POST /api/ctrl            -> {mode: APP|SDK} 停止/启动 genisom_bridge
  POST /api/cmd_vel         -> {vx,vy,wz} 网页遥控（绕过 rosbridge）
  POST /api/video/stop      -> 停止 ffmpeg 图传拉流（省 CPU）
"""
import base64
import json
import math
import os
import re
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
from urllib.parse import unquote
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist, Quaternion, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String, Empty, Float32MultiArray
from sensor_msgs.msg import LaserScan, BatteryState
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time

ROS_SETUP = '/opt/ros/humble/setup.bash'
WS_SETUP = '/home/linaro/robot_ws/install/setup.bash'
MAPS_DIR = '/home/linaro/robot_ws/maps'
SUPERLIO_PCD = '/home/linaro/robot_ws/src/SUPER_LIO/super_lio/map/map.pcd'
SUPERLIO_PCD_DIR = '/home/linaro/robot_ws/src/SUPER_LIO/super_lio/map'
PCD2PGM = '/home/linaro/robot_ws/src/nav2_tools/nav2_tools/pcd2pgm.py'
if not os.path.isfile(PCD2PGM):
    PCD2PGM = '/home/linaro/robot_ws/src/nav2_tools/pcd2pgm.py'
NAV2_PARAMS = '/home/linaro/robot_ws/src/nav2_tools/nav2_params.yaml'
START_ALL = '/home/linaro/robot_ws/start_all.sh'
START_SCAN = '/home/linaro/robot_ws/start_scan_node.sh'
# 狗身广角相机（AgiBot 文档）：有线网段 RTSP H.264
DOG_RTSP = os.environ.get('DOG_RTSP', 'rtsp://192.168.168.168:8554/test')
DOG_FRAME_JPG = '/tmp/dog_live.jpg'
# 默认档（无负载时的高清）；实际以 /api/video/profile 为准
VIDEO_WIDTH = int(os.environ.get('DOG_VIDEO_WIDTH', '1280'))
VIDEO_FPS = int(os.environ.get('DOG_VIDEO_FPS', '15'))
VIDEO_Q = int(os.environ.get('DOG_VIDEO_Q', '3'))
VIDEO_IDLE_SEC = float(os.environ.get('DOG_VIDEO_IDLE_SEC', '45'))
# smooth=导航/建图时保流畅；balanced/best=空闲时按网络探测
VIDEO_PROFILES = {
    'smooth': {'width': 640, 'fps': 12, 'q': 5, 'label': '流畅'},
    'balanced': {'width': 960, 'fps': 12, 'q': 4, 'label': '均衡'},
    'best': {'width': 1280, 'fps': 15, 'q': 3, 'label': '高清'},
}
_video_feeder = {'proc': None, 'lock': threading.Lock(), 'last_use': 0.0}
_video_profile = 'best'
_video_cfg = dict(VIDEO_PROFILES['best'])
# 遥控优先：stop 后一段时间内禁止再拉起 ffmpeg
_video_block_until = 0.0


def _ffmpeg_dog_live_pids():
    """所有写 /tmp/dog_live.jpg 的 ffmpeg（含 web_ops 重启后的孤儿）。"""
    out = []
    me = os.getpid()
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        pid = int(name)
        if pid == me:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode('utf-8', 'ignore')
        except Exception:
            continue
        if 'ffmpeg' in cmd and 'dog_live.jpg' in cmd:
            out.append(pid)
    return out


def stop_video_feeder(block_sec=20.0):
    global _video_block_until
    with _video_feeder['lock']:
        if block_sec and block_sec > 0:
            _video_block_until = max(_video_block_until, time.monotonic() + float(block_sec))
        proc = _video_feeder['proc']
        _video_feeder['proc'] = None
        for pid in _ffmpeg_dog_live_pids():
            try:
                os.kill(pid, signal.SIGTERM)
            except Exception:
                pass
        try:
            if proc is not None and proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
        except Exception:
            pass
    time.sleep(0.05)
    for pid in _ffmpeg_dog_live_pids():
        try:
            os.kill(pid, signal.SIGKILL)
        except Exception:
            pass


def video_stack_busy():
    """建图/导航/LIO 占核时，禁止高清图传把网页拖死。"""
    if proc_alive('component_container_isolated'):
        return True
    if proc_alive('map_building_node'):
        return True
    try:
        if _mapping_session:
            return True
    except NameError:
        pass
    # 仅定位用的 Super-LIO 也很重，运控只能走流畅档
    if proc_alive('super_lio_node'):
        return True
    return False


def set_video_profile(name='best', rtt_ms=None):
    """切换图传档位。smooth=流畅；balanced=均衡；best/auto=按 rtt_ms 选最优。"""
    global _video_profile, _video_cfg
    raw = (name or 'best').strip().lower()
    if raw in ('fluent', 'fluid', 'low', 'lite'):
        raw = 'smooth'
    if raw in ('high', 'hd', 'quality'):
        raw = 'best'
    if raw in ('mid', 'medium', 'normal'):
        raw = 'balanced'
    forced = False
    if video_stack_busy():
        # 负载中强制流畅，忽略客户端要高清
        raw = 'smooth'
        forced = True
    elif raw in ('auto', 'best'):
        pick = 'best'
        if rtt_ms is not None:
            try:
                rtt = float(rtt_ms)
            except (TypeError, ValueError):
                rtt = None
            if rtt is not None:
                if rtt > 1100:
                    pick = 'smooth'
                elif rtt > 600:
                    pick = 'balanced'
                else:
                    pick = 'best'
        if raw == 'auto':
            raw = pick
        else:
            raw = 'balanced' if pick == 'smooth' else pick
    if raw not in VIDEO_PROFILES:
        raw = 'best'
    new_cfg = dict(VIDEO_PROFILES[raw])
    changed = (raw != _video_profile) or (new_cfg != _video_cfg)
    _video_profile = raw
    _video_cfg = new_cfg
    restarted = False
    if changed:
        stop_video_feeder(block_sec=0)
        restarted = bool(ensure_video_feeder())
    return {
        'ok': True,
        'profile': _video_profile,
        'label': VIDEO_PROFILES[_video_profile]['label'],
        'width': _video_cfg['width'],
        'fps': _video_cfg['fps'],
        'q': _video_cfg['q'],
        'restarted': bool(changed),
        'feeder': restarted,
        'rtt_ms': rtt_ms,
        'forced_smooth': forced,
    }


def ensure_video_feeder():
    """Keep one ffmpeg writing latest JPEG；仅在有人看视频且未被遥控暂停时。"""
    global _video_block_until
    with _video_feeder['lock']:
        if time.monotonic() < _video_block_until:
            return False
        _video_feeder['last_use'] = time.monotonic()
        proc = _video_feeder['proc']
        live = _ffmpeg_dog_live_pids()
        if proc is not None and proc.poll() is None and len(live) == 1 and proc.pid in live:
            return True
        # 清理全部残留，保证只留一个
        for pid in live:
            try:
                os.kill(pid, signal.SIGTERM)
            except Exception:
                pass
        time.sleep(0.2)
        for pid in _ffmpeg_dog_live_pids():
            try:
                os.kill(pid, signal.SIGKILL)
            except Exception:
                pass
        try:
            if proc is not None and proc.poll() is None:
                os.killpg(proc.pid, signal.SIGKILL)
        except Exception:
            pass
        try:
            if os.path.isfile(DOG_FRAME_JPG):
                os.remove(DOG_FRAME_JPG)
        except Exception:
            pass
        if time.monotonic() < _video_block_until:
            return False
        cfg = _video_cfg or VIDEO_PROFILES['best']
        w = max(320, min(1280, int(cfg.get('width', VIDEO_WIDTH))))
        fps = max(5, min(20, int(cfg.get('fps', VIDEO_FPS))))
        q = max(2, min(8, int(cfg.get('q', VIDEO_Q))))
        cmd = [
            'ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error',
            '-rtsp_transport', 'tcp',
            '-fflags', 'nobuffer', '-flags', 'low_delay',
            '-probesize', '32', '-analyzeduration', '0',
            '-i', DOG_RTSP,
            '-an',
            '-r', str(fps),
            '-vf', f'scale={w}:-2',
            '-q:v', str(q),
            '-f', 'image2', '-update', '1',
            DOG_FRAME_JPG,
        ]
        try:
            _video_feeder['proc'] = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL, preexec_fn=os.setsid)
            _video_feeder['last_use'] = time.monotonic()
            return True
        except Exception:
            _video_feeder['proc'] = None
            return False


def video_idle_reaper():
    """无人看视频超过 VIDEO_IDLE_SEC 则停 ffmpeg，省 CPU。"""
    while True:
        time.sleep(5.0)
        try:
            with _video_feeder['lock']:
                last = _video_feeder['last_use']
                proc = _video_feeder['proc']
            if last <= 0:
                continue
            if (time.monotonic() - last) < VIDEO_IDLE_SEC:
                continue
            if proc is None and not _ffmpeg_dog_live_pids():
                continue
            stop_video_feeder()
        except Exception:
            pass


def read_live_frame(wait_sec=2.5):
    """Return latest complete JPEG bytes from feeder file."""
    ensure_video_feeder()
    deadline = time.time() + wait_sec
    while time.time() < deadline:
        try:
            if os.path.isfile(DOG_FRAME_JPG) and os.path.getsize(DOG_FRAME_JPG) > 2000:
                with open(DOG_FRAME_JPG, 'rb') as f:
                    data = f.read()
                # 完整 JPEG：SOI + EOI，避免读到半帧
                if data[:2] == b'\xff\xd8' and data[-2:] == b'\xff\xd9':
                    return data
        except Exception:
            pass
        time.sleep(0.025)
    return None


def run_bg(cmd, logfile):
    full = f'source {ROS_SETUP} && source {WS_SETUP} && {cmd}'
    with open(logfile, 'a') as f:
        f.write(f'\n==== {time.strftime("%H:%M:%S")} ====\n')
    subprocess.Popen(['setsid', 'bash', '-c', full],
                     stdout=open(logfile, 'a'), stderr=subprocess.STDOUT,
                     stdin=subprocess.DEVNULL, start_new_session=True)


def proc_pids(pattern):
    """Return PIDs of real processes matching pattern (not agent/shell noise)."""
    pids = []
    me = os.getpid()
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        pid = int(name)
        if pid == me:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode('utf-8', 'ignore')
        except Exception:
            continue
        if pattern not in cmd:
            continue
        # Ignore Cursor/agent shells, pgrep, and this helper text
        if any(x in cmd for x in (
                'extglob', 'pgrep', 'cursor-agent', 'start_all.sh',
                'COMMAND_EXIT_CODE', 'dump_bash_state')):
            continue
        try:
            exe = os.readlink(f'/proc/{pid}/exe')
        except Exception:
            exe = ''
        # Prefer real binaries / installed node entrypoints
        ok = False
        if pattern in exe:
            ok = True
        elif f'/{pattern}' in cmd or f' {pattern} ' in f' {cmd} ':
            # python entrypoints e.g. .../lib/nav2_tools/map_building_node
            if 'ros2 launch' in cmd and pattern in cmd:
                ok = True
            if f'lib/' in cmd and pattern in cmd:
                ok = True
            exe_base = os.path.basename(exe)
            if exe_base.startswith('python3') or exe_base.startswith('python'):
                # only if argv looks like the node, not a random script quoting the name
                # (strip .py so script-style launches like nav_scan_node.py still match)
                parts = cmd.split()
                stems = [os.path.basename(p.rstrip('/')) for p in parts]
                stems = [s[:-3] if s.endswith('.py') else s for s in stems]
                if any(s == pattern or s.endswith(pattern) for s in stems):
                    ok = True
        if ok:
            pids.append(pid)
    return pids


def proc_alive(pattern):
    return bool(proc_pids(pattern))


def kill_pattern(pattern, sig=signal.SIGTERM):
    for pid in proc_pids(pattern):
        try:
            os.kill(pid, sig)
        except Exception:
            pass


# ---------------- HTTP API ----------------
class ApiHandler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _cors(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET,POST,OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def _send_json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self._cors()
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(200)
        self._cors()
        self.end_headers()

    def do_GET(self):
        path = self.path.split('?')[0]
        query = {}
        if '?' in self.path:
            for kv in self.path.split('?')[1].split('&'):
                if '=' in kv:
                    k, v = kv.split('=', 1)
                    query[k] = v
        try:
            if path == '/api/maps':
                self._send_json(list_maps())
            elif path == '/api/map/pgm':
                name = query.get('name', '')
                self._serve_pgm(name)
            elif path == '/api/map/preview':
                name = query.get('name', '')
                self._serve_map_preview(name)
            elif path == '/api/sysinfo':
                self._send_json(sysinfo())
            elif path == '/api/battery':
                self._send_json(battery_json())
            elif path == '/api/mapping/status':
                self._send_json(mapping_status())
            elif path == '/api/lidar_extrinsic':
                self._send_json(load_lidar_extrinsic())
            elif path == '/api/nav2/map':
                self._send_json(current_map_json())
            elif path == '/api/log':
                file = query.get('file', 'webops.log')
                n = int(query.get('n', '50'))
                self._send_json({'file': file, 'log': tail_log(file, n)})
            elif path == '/api/video/mjpeg':
                self._serve_mjpeg(query.get('url') or DOG_RTSP)
            elif path == '/api/video/snapshot':
                self._serve_snapshot(query.get('url') or DOG_RTSP)
            else:
                self._send_json({'error': 'not found'}, 404)
        except Exception as e:
            self._send_json({'error': str(e)}, 500)

    def do_POST(self):
        path = self.path.split('?')[0]
        try:
            ln = int(self.headers.get('Content-Length', 0))
            raw = self.rfile.read(ln) if ln else b''
            body = json.loads(raw.decode('utf-8')) if raw else {}
            if path == '/api/map/edit':
                self._send_json(save_map_edit(body))
            elif path == '/api/map/delete':
                self._send_json(delete_map(body))
            elif path == '/api/mapping/save':
                self._send_json(save_mapping(body))
            elif path == '/api/mapping/stop':
                self._send_json(stop_mapping_keep_pcd())
            elif path == '/api/mapping/discard':
                self._send_json(discard_mapping())
            elif path == '/api/mapping/start':
                self._send_json(start_mapping_session(body))
            elif path == '/api/nav2/start':
                self._send_json(start_nav2(body))
            elif path == '/api/nav2/stop':
                self._send_json(stop_nav2_api())
            elif path == '/api/nav2/relocalize':
                self._send_json(call_relocalize())
            elif path == '/api/nav2/global_relocalize':
                self._send_json(global_relocalize())
            elif path == '/api/nav2/accept_pose':
                self._send_json(accept_reloc_pose())
            elif path == '/api/nav2/reloc_mode':
                # 已取消双模式：仅手动重定位（点按钮才搜图）
                self._send_json({'ok': True, 'mode': 'manual', 'note': '仅手动重定位：点「重定位」按钮执行'})
            elif path == '/api/nav2/reload_map':
                self._send_json(reload_nav_map(body))
            elif path == '/api/restart_rosbridge':
                self._send_json(restart_rosbridge())
            elif path == '/api/ctrl':
                self._send_json(set_ctrl_mode(body.get('mode', '')))
            elif path == '/api/cmd_vel':
                self._send_json(publish_cmd_vel(body))
            elif path == '/api/video/stop':
                # 运动页遥控短暂停图；建图页前端不再调此接口
                stop_video_feeder(block_sec=6.0)
                self._send_json({'ok': True, 'stopped': True, 'blocked_sec': 6})
            elif path == '/api/video/resume':
                global _video_block_until
                _video_block_until = 0.0
                self._send_json({'ok': True, 'resumed': True})
            elif path == '/api/video/profile':
                self._send_json(set_video_profile(
                    body.get('profile', 'best'), body.get('rtt_ms')))
            else:
                self._send_json({'error': 'not found'}, 404)
        except Exception as e:
            self._send_json({'error': str(e)}, 500)

    def _serve_pgm(self, name):
        safe = os.path.basename(name)
        p = os.path.join(MAPS_DIR, safe)
        if not safe.endswith('.pgm') or not os.path.isfile(p):
            self._send_json({'error': 'no such map'}, 404)
            return
        with open(p, 'rb') as f:
            data = f.read()
        self.send_response(200)
        self.send_header('Content-Type', 'image/x-portable-graymap')
        self._cors()
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _serve_map_preview(self, name):
        """Serve PGM as PNG — browsers cannot render raw PGM in <img>."""
        safe = os.path.basename(unquote(name))
        if not safe.endswith('.pgm'):
            safe += '.pgm'
        p = os.path.join(MAPS_DIR, safe)
        if not os.path.isfile(p):
            self._send_json({'error': 'no such map'}, 404)
            return
        try:
            png = pgm_file_to_png(p)
        except Exception as e:
            self._send_json({'error': str(e)}, 500)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'image/png')
        self.send_header('Cache-Control', 'no-cache')
        self._cors()
        self.send_header('Content-Length', str(len(png)))
        self.end_headers()
        self.wfile.write(png)

    def _serve_mjpeg(self, rtsp_url):
        """Serve multipart MJPEG from live frame cache (browser-friendly)."""
        if time.monotonic() < _video_block_until:
            self._send_json({'error': 'video paused for teleop', 'blocked': True}, 503)
            return
        ensure_video_feeder()
        try:
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Access-Control-Allow-Origin', '*')
            self._cors()
            self.end_headers()
            last = b''
            idle = 0
            interval = max(0.04, 1.0 / max(8, min(30, int((_video_cfg or {}).get('fps', VIDEO_FPS)))))
            while idle < 100:
                if time.monotonic() < _video_block_until:
                    break
                data = read_live_frame(wait_sec=0.5)
                if not data:
                    idle += 1
                    time.sleep(0.05)
                    continue
                idle = 0
                if data == last:
                    time.sleep(0.02)
                    continue
                last = data
                part = (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n'
                    b'Content-Length: ' + str(len(data)).encode() + b'\r\n\r\n'
                    + data + b'\r\n'
                )
                self.wfile.write(part)
                self.wfile.flush()
                time.sleep(interval * 0.5)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception:
            pass

    def _serve_snapshot(self, rtsp_url):
        # 遥控 block 期间不要再起 ffmpeg（哪怕单帧），否则建图页轮询会打满 CPU
        if time.monotonic() < _video_block_until:
            self._send_json({'error': 'video paused for teleop', 'blocked': True}, 503)
            return
        data = read_live_frame(wait_sec=1.5)
        if not data:
            if time.monotonic() < _video_block_until:
                self._send_json({'error': 'video paused for teleop', 'blocked': True}, 503)
                return
            cmd = [
                'ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error',
                '-rtsp_transport', 'tcp', '-fflags', 'nobuffer', '-flags', 'low_delay',
                '-y', '-i', rtsp_url or DOG_RTSP,
                '-frames:v', '1', '-q:v', str(max(2, min(8, int((_video_cfg or {}).get('q', VIDEO_Q))))),
                '-vf', f'scale={max(320, min(1280, int((_video_cfg or {}).get("width", VIDEO_WIDTH))))}:-2',
                '-f', 'image2pipe', '-vcodec', 'mjpeg', '-',
            ]
            try:
                r = subprocess.run(cmd, capture_output=True, timeout=8)
                data = r.stdout if r.returncode == 0 else b''
            except Exception:
                data = b''
        if not data:
            self._send_json({'error': 'snapshot failed'}, 502)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Cache-Control', 'no-cache, no-store')
        self._cors()
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def pgm_file_to_png(path, max_side=256):
    """Read a P5 PGM and return PNG bytes suitable for <img> thumbnails."""
    with open(path, 'rb') as f:
        raw = f.read()
    i = 0
    n = len(raw)

    def tok():
        nonlocal i
        while i < n:
            if raw[i:i + 1] == b'#':
                while i < n and raw[i:i + 1] != b'\n':
                    i += 1
                continue
            if raw[i] in (9, 10, 13, 32):
                i += 1
                continue
            break
        s = b''
        while i < n and raw[i] not in (9, 10, 13, 32) and raw[i:i + 1] != b'#':
            s += raw[i:i + 1]
            i += 1
        return s.decode('ascii', 'replace')

    if tok() != 'P5':
        raise ValueError('not a P5 PGM')
    w, h, _mv = int(tok()), int(tok()), int(tok())
    while i < n and raw[i] in (9, 10, 13, 32):
        i += 1
    pix = raw[i:i + w * h]
    if len(pix) < w * h:
        raise ValueError('truncated PGM')
    # ROS PGM: 0 障碍 / 254 自由 / 205 未知 —— 不可用 >200，否则 205 会被当成自由
    out = bytearray(w * h)
    for k, v in enumerate(pix):
        out[k] = 0 if v <= 50 else (255 if v >= 250 else 128)
    img = Image.frombytes('L', (w, h), bytes(out)).convert('RGB')
    if max(w, h) > max_side:
        img.thumbnail((max_side, max_side), Image.NEAREST)
    buf = BytesIO()
    img.save(buf, format='PNG', optimize=True)
    return buf.getvalue()


def list_maps():
    out = []
    if not os.path.isdir(MAPS_DIR):
        return {'maps': out}
    for f in sorted(os.listdir(MAPS_DIR)):
        if not f.endswith('.pgm'):
            continue
        if f.startswith('__') or '.bak.' in f or f.endswith('.bak.pgm'):
            continue
        yaml_path = os.path.join(MAPS_DIR, f[:-4] + '.yaml')
        meta = {'resolution': 0.05, 'origin': [0.0, 0.0, 0.0]}
        if os.path.isfile(yaml_path):
            for line in open(yaml_path):
                m = re.match(r'resolution:\s*([\d.eE+-]+)', line)
                if m:
                    meta['resolution'] = float(m.group(1))
                m = re.match(r'origin:\s*\[([^,]+),\s*([^,\]]+)', line)
                if m:
                    meta['origin'] = [float(m.group(1)), float(m.group(2)), 0.0]
        try:
            with open(os.path.join(MAPS_DIR, f), 'rb') as fh:
                head = fh.read(64)
            m = re.match(rb'P5\n(\d+) (\d+)\n', head)
            w, h = (int(m.group(1)), int(m.group(2))) if m else (0, 0)
        except Exception:
            w = h = 0
        out.append({'name': f, 'width': w, 'height': h,
                    'resolution': meta['resolution'], 'origin': meta['origin'],
                    'mtime': time.strftime('%m-%d %H:%M', time.localtime(
                        os.path.getmtime(os.path.join(MAPS_DIR, f))))})
    return {'maps': out}


def save_map_edit(body):
    name = os.path.basename(body.get('name', ''))
    if not name.endswith('.pgm'):
        name += '.pgm'
    data = base64.b64decode(body['data'])
    pgm_path = os.path.join(MAPS_DIR, name)
    with open(pgm_path, 'wb') as f:
        f.write(data)
    res = float(body.get('resolution', 0.05))
    origin = body.get('origin', [0.0, 0.0])
    yaml_path = pgm_path[:-4] + '.yaml'
    with open(yaml_path, 'w') as f:
        f.write('image: %s\n' % name)
        f.write('resolution: %f\n' % res)
        f.write('origin: [%f, %f, 0.0]\n' % (float(origin[0]), float(origin[1])))
        f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.15\n')
    return {'ok': True, 'name': name}


def delete_map(body):
    name = os.path.basename(body.get('name', ''))
    if not name.endswith('.pgm'):
        return {'ok': False, 'error': 'name must end .pgm'}
    stem = name[:-4]
    # 当前导航选中的图 / Nav2 正在加载的图：禁止删，避免「库里没了但系统还挂着旧图」
    selected = body.get('selected') or body.get('sel_map') or ''
    selected = os.path.basename(str(selected))
    if selected.endswith(('.pgm', '.yaml')):
        selected = selected.rsplit('.', 1)[0]
    if selected and selected == stem:
        return {'ok': False, 'error': '「%s」是当前导航地图，不能删除；请先换成其它图再删' % name}
    cur = _nav2_current_map or ''
    if cur and (os.path.basename(cur) == stem + '.yaml'
                or os.path.basename(cur) == stem + '.pgm'):
        return {'ok': False, 'error': 'Nav2 正在使用「%s」，请先关闭导航或换图后再删' % name}
    removed = []
    for suffix in ('.pgm', '.yaml'):
        p = os.path.join(MAPS_DIR, stem + suffix)
        if os.path.isfile(p):
            os.remove(p)
            removed.append(suffix)
    if not removed:
        return {'ok': False, 'error': '地图文件不存在'}
    return {'ok': True, 'removed': removed}


def stop_relocation_node(sig=signal.SIGTERM):
    """停掉先验重定位节点（与 super_lio_node 互斥，同发 /lio/odom）。"""
    killed = []
    for needle in ('relocation_node', 'dog_reloc.py'):
        for pid in proc_pids(needle):
            try:
                os.kill(pid, sig)
                killed.append(pid)
            except Exception:
                pass
    return killed


def collapse_map_building(keep_one=True):
    """叠了多个 map_building_node 时只留最新一个（或全停）。"""
    pids = sorted(proc_pids('map_building_node'))
    launches = sorted(proc_pids('map_building.launch'))
    if keep_one and len(pids) <= 1 and len(launches) <= 1:
        return {'ok': True, 'kept': pids[:1], 'killed': []}
    killed = []
    # 先杀多余 launch，再杀多余 node
    drop_l = launches[:-1] if (keep_one and launches) else launches
    drop_n = pids[:-1] if (keep_one and pids) else pids
    if not keep_one:
        drop_l, drop_n = launches, pids
    for pid in drop_l + drop_n:
        try:
            os.kill(pid, signal.SIGTERM)
            killed.append(pid)
        except Exception:
            pass
    time.sleep(0.4)
    for pid in drop_l + drop_n:
        try:
            os.kill(pid, signal.SIGKILL)
        except Exception:
            pass
    return {'ok': True, 'kept': ([] if not keep_one else sorted(proc_pids('map_building_node'))[-1:]),
            'killed': killed}


def stop_super_lio(sig=signal.SIGINT):
    pids = proc_pids('super_lio_node')
    for pid in pids:
        try:
            os.kill(pid, sig)
        except Exception:
            pass
    return pids


PCD_DIR = os.path.dirname(SUPERLIO_PCD)
LIDAR_EXTRINSIC = '/home/linaro/robot_ws/config/lidar_extrinsic.yaml'

def load_lidar_extrinsic():
    """Read pitch_down / roll / lidar_z for UI + pcd2pgm."""
    out = {
        'ok': True,
        'pitch_down': 0.342586,  # dog-head Mid360 looking down ~19.6°
        'roll': 0.0,
        'lidar_z': 0.45,
        'lidar_pitch': -0.342586,
        'mount_pitch_down': 0.342586,
        'source': 'default',
    }
    if not os.path.isfile(LIDAR_EXTRINSIC):
        return out
    try:
        with open(LIDAR_EXTRINSIC, 'r') as f:
            for ln in f:
                ln = ln.split('#', 1)[0].strip()
                if ':' not in ln:
                    continue
                k, v = ln.split(':', 1)
                k, v = k.strip(), v.strip()
                try:
                    out[k] = float(v)
                except ValueError:
                    out[k] = v
        if 'pitch_down' not in out and 'lidar_pitch' in out:
            out['pitch_down'] = abs(float(out['lidar_pitch']))
        out['ok'] = True
        out['source'] = 'file'
    except Exception as e:
        out['ok'] = False
        out['error'] = str(e)
    return out


def clear_pcd_cache():
    """删除 Super-LIO 上次留下的 PCD（map.pcd + map/PCD/scans_*.pcd），避免旧云污染新图。"""
    removed = []
    try:
        if os.path.isdir(SUPERLIO_PCD_DIR):
            for fn in os.listdir(SUPERLIO_PCD_DIR):
                p = os.path.join(SUPERLIO_PCD_DIR, fn)
                if fn == os.path.basename(SUPERLIO_PCD) or fn == 'PCD':
                    try:
                        if os.path.isdir(p):
                            for sub in os.listdir(p):
                                sp = os.path.join(p, sub)
                                if sub.startswith('scans') and sub.endswith('.pcd'):
                                    try:
                                        os.remove(sp)
                                        removed.append('PCD/' + sub)
                                    except Exception:
                                        pass
                        else:
                            os.remove(p)
                            removed.append(fn)
                    except Exception:
                        pass
    except Exception as e:
        return {'ok': False, 'error': str(e), 'removed': removed}
    return {'ok': True, 'removed': removed}


def pcd_ready():
    """是否有可转换的 map.pcd（体积足够）。"""
    try:
        return os.path.isfile(SUPERLIO_PCD) and os.path.getsize(SUPERLIO_PCD) > 1024
    except Exception:
        return False


def mapping_status():
    global _mapping_session, _mapping_session_known
    running = proc_alive('super_lio_node')
    building = proc_alive('map_building_node')
    nav2 = proc_alive('component_container_isolated')
    # 仅 web_ops 重启后、尚无显式 start/stop 时，用 LIO+预览推断「正在建图」
    if (not _mapping_session_known and running and building and not nav2
            and not _mapping_stopping):
        _mapping_session = True
        _mapping_session_known = True
    if nav2 or _mapping_stopping:
        session = False
    else:
        session = bool(_mapping_session)
    ready = pcd_ready()
    size = 0
    mtime = None
    if os.path.isfile(SUPERLIO_PCD):
        try:
            size = os.path.getsize(SUPERLIO_PCD)
            mtime = time.strftime('%H:%M:%S', time.localtime(os.path.getmtime(SUPERLIO_PCD)))
        except Exception:
            pass
    return {
        'ok': True,
        'session': session,
        'stopping': bool(_mapping_stopping),
        'fastlio': running,
        'map_building': building,
        'pcd_ready': (not running) and ready,
        'pcd_exists': ready,
        'pcd_size': size,
        'pcd_mtime': mtime,
        'can_start': not session and not running and not nav2 and not _mapping_stopping,
        'can_stop': (session or running) and not _mapping_stopping,
        'can_save': (not running) and ready and not _mapping_stopping,
    }


def stop_mapping_keep_pcd():
    """停止 Super-LIO：SIGINT 触发 saveMap() 落盘 map.pcd，再等待生成。"""
    global _mapping_stopping
    set_mapping_session(False)
    _mapping_stopping = True
    try:
        pids = proc_pids('super_lio_node')
        if not pids:
            return {
                'ok': True, 'already_stopped': True,
                'pcd_ready': pcd_ready(),
                'pcd_size': os.path.getsize(SUPERLIO_PCD) if os.path.isfile(SUPERLIO_PCD) else 0,
            }

        prev_mtime = os.path.getmtime(SUPERLIO_PCD) if os.path.isfile(SUPERLIO_PCD) else 0
        stop_super_lio(signal.SIGINT)
        saved = False
        for _ in range(30):
            if os.path.isfile(SUPERLIO_PCD):
                mt = os.path.getmtime(SUPERLIO_PCD)
                if mt > prev_mtime or (time.time() - mt) < 60:
                    time.sleep(0.5)
                    if os.path.getsize(SUPERLIO_PCD) > 1024:
                        saved = True
                        break
            time.sleep(0.5)
        kill_pattern('map_building_node')
        # 一并停掉 launch，避免子进程退出后被 launch 异常拉起
        kill_pattern('Livox_mid360.py')
        still = proc_pids('super_lio_node')
        # Only force-kill if PCD already on disk; otherwise give a bit more time
        if still and not saved:
            time.sleep(2.0)
            if os.path.isfile(SUPERLIO_PCD) and os.path.getsize(SUPERLIO_PCD) > 1024:
                saved = True
        for pid in still:
            try:
                os.kill(pid, signal.SIGKILL)
            except Exception:
                pass
        # 确保会话保持关闭（防止等待期间被其它逻辑改写）
        set_mapping_session(False)
        return {
            'ok': True,
            'pcd_ready': pcd_ready(),
            'pcd_saved': saved or pcd_ready(),
            'pcd_size': os.path.getsize(SUPERLIO_PCD) if os.path.isfile(SUPERLIO_PCD) else 0,
            'force_killed': bool(still),
            'error': None if (saved or pcd_ready()) else '未生成 map.pcd，请确认已在 livox_360.yaml 开启 lio.map.save_map 后重新建图',
        }
    finally:
        _mapping_stopping = False
        set_mapping_session(False)


def start_mapping_session(body=None):
    """清理旧 PCD 后启动/确保建图链（幂等，不会叠多个 Livox）。

    真正的幂等靠 super_lio_node/map_building_node 是否已存活来判断，但
    run_bg() 是异步的——从"没看到进程"到"进程真的起来"之间有好几秒空窗。
    两次几乎同时的 /api/mapping/start 请求（双击、页面重试、脚本和网页撞车）
    都会在这个空窗里各自判定"没在跑"然后各起一份 start_all.sh mapping，
    而 start_all.sh 自己的 ensure_one() 也是同样的检查-再启动，两边一起
    竞态就会叠出两个 super_lio_node + 两个 map_building_node 抢同一路雷达，
    建图直接乱掉（这次排查到的真实故障)。用一把进程内锁把"决定要不要启动"
    这段收窄到互斥执行，第二个请求要么等第一个判定完直接复用，要么在第一个
    还没判定完时立刻被拒绝，不会再有两边同时通过检查的窗口。
    """
    global _mapping_starting
    with _mapping_lock:
        if _mapping_stopping:
            return {'ok': False, 'error': '正在停止建图，请稍候'}
        if _mapping_starting:
            return {'ok': False, 'error': '建图正在启动中，请稍候'}
        if proc_alive('component_container_isolated'):
            return {'ok': False, 'error': '请先关闭导航'}
        # 与 relocation_node 互斥，否则 /lio/odom 双源导致建图页狗位姿跳变
        if proc_alive('relocation_node'):
            stop_relocation_node()
            time.sleep(0.8)
        # 若已在建图，仅确保预览（并压扁叠出来的多个 map_building）
        if proc_alive('super_lio_node'):
            set_mapping_session(True)
            collapse_map_building(keep_one=True)
            if not proc_alive('map_building_node'):
                run_bg(
                    'ros2 launch nav2_tools map_building.launch.py',
                    '/tmp/map_building.log')
            return {'ok': True, 'already_running': True, 'cleared': [], 'session': True}
        _mapping_starting = True
    try:
        cleared = clear_pcd_cache()
        # 再清一次，防止锁外窗口又被 start_reloc 拉起
        stop_relocation_node()
        collapse_map_building(keep_one=False)
        # start_all.sh is idempotent: collapses duplicate Livox/LIO first
        run_bg(f'bash {START_ALL} mapping', '/tmp/mapping.log')
        # 等 start_all.sh 里的 ensure_one() 真正把 super_lio_node 跑起来再放锁，
        # 覆盖掉 run_bg() 异步返回到进程实际存活之间的那段窗口。
        for _ in range(30):
            if proc_alive('super_lio_node'):
                break
            time.sleep(0.5)
        set_mapping_session(True)
        collapse_map_building(keep_one=True)
        return {'ok': True, 'started': True, 'cleared': cleared.get('removed', []), 'session': True}
    finally:
        _mapping_starting = False


def discard_mapping():
    """Stop Super-LIO without converting PCD to a map."""
    set_mapping_session(False)
    pids = stop_super_lio(signal.SIGINT)
    # Also stop map_building preview node so next session starts clean
    kill_pattern('map_building_node')
    time.sleep(0.5)
    if pids:
        # Force if still alive
        still = proc_pids('super_lio_node')
        for pid in still:
            try:
                os.kill(pid, signal.SIGKILL)
            except Exception:
                pass
    return {'ok': True, 'stopped': len(pids), 'discarded': True}


def save_mapping(body):
    """保存建图：SIGINT super_lio -> 等 map.pcd -> pcd2pgm -> maps/<name>.pgm+yaml"""
    name = os.path.basename(body.get('name', 'map'))
    if name in ('__discard__', 'discard', ''):
        return discard_mapping()
    if name.endswith(('.pgm', '.yaml')):
        name = name.rsplit('.', 1)[0]
    name = re.sub(r'[^\w\-]+', '_', name) or 'map'

    set_mapping_session(False)
    pids = proc_pids('super_lio_node')
    if not pids:
        if os.path.isfile(SUPERLIO_PCD):
            return run_pcd2pgm(name)
        return {'ok': False, 'error': 'super_lio 未运行且无 PCD 可转换'}

    # Remember mtime before kill so we wait for a fresh PCD
    prev_mtime = os.path.getmtime(SUPERLIO_PCD) if os.path.isfile(SUPERLIO_PCD) else 0
    stop_super_lio(signal.SIGINT)

    pcd_path = SUPERLIO_PCD
    saved = False
    for _ in range(20):
        if os.path.isfile(pcd_path):
            mt = os.path.getmtime(pcd_path)
            if mt > prev_mtime or (time.time() - mt) < 30:
                # Wait a bit more for flush
                time.sleep(1.0)
                if os.path.getsize(pcd_path) > 100:
                    saved = True
                    break
        time.sleep(1)

    kill_pattern('map_building_node')
    if not saved and not os.path.isfile(pcd_path):
        return {'ok': False, 'error': 'Super-LIO 已停止但未生成 map.pcd'}
    return run_pcd2pgm(name, pcd_path)


def run_pcd2pgm(name, pcd_path=SUPERLIO_PCD):
    import tempfile
    import shutil
    os.makedirs(MAPS_DIR, exist_ok=True)
    out_base = os.path.join(MAPS_DIR, name)
    overwritten = os.path.isfile(out_base + '.pgm') or os.path.isfile(out_base + '.yaml')
    # 大 PCD（建图走久会到 GB 级）必须体素降采样，否则 python 列表 OOM → 被杀 →「pgm not written」
    try:
        pcd_sz = os.path.getsize(pcd_path) if os.path.isfile(pcd_path) else 0
    except OSError:
        pcd_sz = 0
    # PCD 过旧时多半是上一次建图残留：仍允许转换，但打标给前端提示
    pcd_age = None
    try:
        pcd_age = time.time() - os.path.getmtime(pcd_path)
    except OSError:
        pass
    voxel = '0.05' if pcd_sz > 200 * 1024 * 1024 else '0.04'
    timeout = 600 if pcd_sz > 400 * 1024 * 1024 else 300
    # 先写到临时目录再原子替换，避免转换失败时留下半截/旧图混用
    tmp_dir = tempfile.mkdtemp(prefix='pcd2pgm_', dir=MAPS_DIR)
    tmp_base = os.path.join(tmp_dir, name)
    cmd = [
        'python3', PCD2PGM, pcd_path, tmp_base,
        # Super-LIO 的 map.pcd 在重力对齐的 world 系（z 向上），
        # 期望地平面法向量为 +Z，故 pitch_down/roll/lidar_z 都取 0（由 RANSAC 精修高度）。
        '--pitch-down', '0',
        '--roll', '0',
        '--lidar-z', '0',
        '--z-min', '0.10', '--z-max', '1.0',
        '--occ-min', '0.12', '--occ-pts', '8',
        '--free-pts', '1', '--ground-min-z', '-0.15',
        '--free-dilate', '8', '--free-erode', '2',
        '--despeckle', '3',
        '--voxel', voxel,
    ]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        log = (r.stdout + r.stderr).strip()
        tmp_pgm = tmp_base + '.pgm'
        tmp_yaml = tmp_base + '.yaml'
        ok = os.path.isfile(tmp_pgm) and os.path.isfile(tmp_yaml)
        if ok:
            for suf in ('.pgm', '.yaml'):
                os.replace(tmp_base + suf, out_base + suf)
            # 若 Nav2 正挂着同名图，热加载新文件，避免界面仍显示内存里的旧栅格
            reloaded = None
            cur = _nav2_current_map or ''
            if cur and os.path.basename(cur) == name + '.yaml':
                reloaded = reload_nav_map({'map': name + '.yaml', 'reloc': False})
            return {
                'ok': True,
                'name': name + '.pgm',
                'overwritten': overwritten,
                'pcd_age_sec': pcd_age,
                'reloaded': reloaded,
                'log': log[-2000:],
                'error': None,
            }
        err = log[-500:] if log else ''
        if r.returncode == -9 or r.returncode == 137:
            err = (err + ' | ').lstrip(' |') + (
                f'转换进程被系统杀掉(OOM)。PCD≈{pcd_sz/1e9:.2f}GB，已用 voxel={voxel}m；'
                '请缩短单次建图或再试一次保存')
        elif not err:
            err = f'pgm not written (exit={r.returncode})'
        return {'ok': False, 'name': name + '.pgm', 'log': log[-2000:], 'error': err}
    except subprocess.TimeoutExpired:
        return {'ok': False, 'error': f'PCD→PGM 超时(>{timeout}s)，点云过大，请缩短建图再保存'}
    except Exception as e:
        return {'ok': False, 'error': str(e)}
    finally:
        try:
            shutil.rmtree(tmp_dir, ignore_errors=True)
        except Exception:
            pass


def _scan_proc_alive():
    """nav_scan_node (leveled height band) or legacy pointcloud_to_laserscan."""
    return proc_alive('nav_scan_node') or proc_alive('pointcloud_to_laserscan')


def ensure_scan_node():
    if proc_alive('nav_scan_node'):
        return True
    # Drop legacy stock laserscan — wrong on pitched Mid360 (tilted base_link slice)
    if proc_alive('pointcloud_to_laserscan'):
        run_bg("pkill -f pointcloud_to_laserscan_node || true", '/tmp/scan_node.log')
        time.sleep(0.4)
    if os.path.isfile(START_SCAN):
        run_bg(f'bash {START_SCAN}', '/tmp/scan_node.log')
        time.sleep(1.5)
    return _scan_proc_alive()


_nav2_lock = threading.Lock()
_nav2_starting = False
_nav2_current_map = None  # 幂等启动用: 同一张图已在跑就别重新盲搜, 见 _start_nav2_impl

_mapping_lock = threading.Lock()
_mapping_starting = False
# 跨设备共享的「建图会话」：任一端点「开始建图」置位，停止/保存/丢弃清掉。
# 另一端只靠本地 mappingSession 会永远显示空闲。
_mapping_session = False
# False=进程启动后尚未被 start/stop 显式写过；仅此时才允许用进程存活推断会话。
# 否则停止过程中 LIO 还在落盘时 status 轮询会把 session 又推断成 True，前端再同步/自动恢复，表现为「关不掉」。
_mapping_session_known = False
_mapping_stopping = False


def set_mapping_session(active):
    global _mapping_session, _mapping_session_known
    _mapping_session = bool(active)
    _mapping_session_known = True


_loc_heal_ts = 0.0


def collapse_lio_tf_bridge(keep_one=True):
    """多个 lio_tf_bridge 会抢发 odom→base_footprint，网页箭头/激光必晃。"""
    bins, launches = [], []
    me = os.getpid()
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        pid = int(name)
        if pid == me:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode('utf-8', 'ignore')
        except Exception:
            continue
        if 'extglob' in cmd or 'COMMAND_EXIT_CODE' in cmd:
            continue
        if 'lio_tf_bridge/lio_tf_bridge' in cmd:
            bins.append(pid)
        elif 'ros2 launch' in cmd and 'lio_tf_bridge' in cmd:
            launches.append(pid)
    if not bins and not launches:
        return {'ok': True, 'kept': 0}
    if keep_one and len(bins) <= 1 and len(launches) <= 1:
        return {'ok': True, 'kept': len(bins)}
    # 只留最新的一个 node + 其 launch；其余杀掉
    bins.sort()
    launches.sort()
    keep_bin = bins[-1] if bins else None
    keep_launch = launches[-1] if launches else None
    for pid in bins + launches:
        if pid in (keep_bin, keep_launch):
            continue
        try:
            os.kill(pid, signal.SIGKILL)
        except Exception:
            pass
    time.sleep(0.4)
    return {'ok': True, 'kept': 1 if keep_bin else 0, 'killed': True}


def ensure_localization_stack(force=False):
    """Nav2 需要 /lio/odom + lio_tf_bridge。

    super_lio_node 与 relocation_node 二选一（都发 /lio/odom）；两者同时在会跳变。
    """
    global _loc_heal_ts
    collapse_lio_tf_bridge(keep_one=True)
    if proc_alive('super_lio_node') and proc_alive('relocation_node'):
        stop_relocation_node()
        time.sleep(0.5)
    has_lio = proc_alive('super_lio_node') or proc_alive('relocation_node')
    need = (not has_lio or not proc_alive('lio_tf_bridge')
            or not proc_alive('livox_ros_driver2_node'))
    if need or force:
        # Cooldown avoids thrashing if LIO keeps dying (OOM / SIGKILL)
        now = time.time()
        if force or (now - _loc_heal_ts) > 8.0:
            _loc_heal_ts = now
            # 统一走 Super-LIO localization；若现场要用 reloc，请先 start_reloc.sh
            stop_relocation_node()
            run_bg(f'bash {START_ALL} localization', '/tmp/localization.log')
            for _ in range(30):
                if proc_alive('super_lio_node') and proc_alive('lio_tf_bridge'):
                    break
                time.sleep(0.4)
            collapse_lio_tf_bridge(keep_one=True)
    return {
        'fastlio': proc_alive('super_lio_node') or proc_alive('relocation_node'),
        'lio_tf': proc_alive('lio_tf_bridge'),
        'livox': proc_alive('livox_ros_driver2_node'),
    }


def heal_nav_deps():
    """If Nav2 is up but LIO/scan/reloc died, bring them back (no dog motion)."""
    if not proc_alive('component_container_isolated'):
        return
    # 不论是否缺进程，先压扁叠出来的桥（双 TF 比缺桥更糟）
    collapse_lio_tf_bridge(keep_one=True)
    if proc_alive('super_lio_node') and proc_alive('relocation_node'):
        stop_relocation_node()
    has_lio = proc_alive('super_lio_node') or proc_alive('relocation_node')
    if not has_lio or not proc_alive('lio_tf_bridge'):
        ensure_localization_stack()
    if not _scan_proc_alive():
        ensure_scan_node()
    if not proc_alive('auto_relocalize'):
        ensure_auto_relocalize()


def stop_nav2_procs():
    for pat in ['component_container_isolated', 'bringup_launch.py',
                'nav2_bringup', 'bt_navigator', 'controller_server',
                'planner_server', 'behavior_server', 'waypoint_follower',
                'velocity_smoother', 'lifecycle_manager', 'auto_relocalize']:
        kill_pattern(pat, signal.SIGKILL)
    time.sleep(1.2)


def ensure_auto_relocalize():
    """保证只有一个 auto_relocalize；重复实例会连环刷 /initialpose 打乱导航。"""
    def _collect():
        bins, launches = [], []
        me = os.getpid()
        for name in os.listdir('/proc'):
            if not name.isdigit():
                continue
            pid = int(name)
            if pid == me:
                continue
            try:
                with open(f'/proc/{pid}/cmdline', 'rb') as f:
                    cmd = f.read().replace(b'\0', b' ').decode('utf-8', 'ignore')
            except Exception:
                continue
            if 'extglob' in cmd or 'COMMAND_EXIT_CODE' in cmd:
                continue
            if 'auto_relocalize/auto_relocalize' in cmd:
                bins.append(pid)
            elif 'auto_relocalize.launch' in cmd or (
                    'ros2 launch' in cmd and 'auto_relocalize' in cmd):
                launches.append(pid)
        return bins, launches

    def _kill_all(bins, launches):
        for pid in bins + launches:
            try:
                os.kill(pid, signal.SIGKILL)
            except Exception:
                pass

    bins, launches = _collect()
    if len(bins) > 1 or len(launches) > 1:
        _kill_all(bins, launches)
        time.sleep(0.8)
        bins, launches = _collect()

    if len(bins) == 1:
        return True

    if bins or launches:
        _kill_all(bins, launches)
        time.sleep(0.5)

    run_bg(
        'ros2 launch auto_relocalize auto_relocalize.launch.py',
        '/tmp/auto_relocalize.log')
    for _ in range(15):
        time.sleep(0.3)
        bins, launches = _collect()
        if len(bins) > 1 or len(launches) > 1:
            _kill_all(bins, launches)
            time.sleep(0.5)
            run_bg(
                'ros2 launch auto_relocalize auto_relocalize.launch.py',
                '/tmp/auto_relocalize.log')
            continue
        if len(bins) == 1:
            return True
    bins, _ = _collect()
    return len(bins) == 1


_web_ops_node = None  # set in main(); HTTP handlers use it to gate nav
_cmd_vel_lock = threading.Lock()
_cmd_vel_seq = 0  # 已接受的最大 seq；用于丢弃乱序晚到的非零指令
_cmd_vel_stop_mono = 0.0  # 最近一次零速的 monotonic 时间


def current_map_json():
    """/map 的 HTTP 直取通道，绕开 rosbridge 对该话题的订阅时序问题（见 cb_map 注释）。

    整形成跟 rosbridge 把 nav_msgs/OccupancyGrid 转成 JSON 时一样的结构，
    前端可以直接把返回的 map 字段扔给现成的 ingest(m,'nav')，不用另外写解析。
    """
    node = _web_ops_node
    msg = node._last_map if node is not None else None
    if msg is None:
        return {'ok': False, 'error': '地图还没加载（Nav2/map_server 可能还没起来）'}
    o = msg.info.origin
    return {
        'ok': True,
        'map': {
            'info': {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin': {
                    'position': {'x': o.position.x, 'y': o.position.y, 'z': o.position.z},
                    'orientation': {'x': o.orientation.x, 'y': o.orientation.y,
                                    'z': o.orientation.z, 'w': o.orientation.w},
                },
            },
            'data': list(msg.data),
        },
    }


def accept_reloc_pose():
    """用户承认当前位姿 / 手动设姿完成 → 锁定看门狗。"""
    script = (
        'source /opt/ros/humble/setup.bash && '
        'source /home/linaro/robot_ws/install/setup.bash && '
        'ros2 service call /accept_reloc_pose std_srvs/srv/Trigger'
    )
    try:
        r = subprocess.run(
            ['bash', '-lc', script],
            capture_output=True, text=True, timeout=8.0)
        out = (r.stdout or '') + (r.stderr or '')
        ok = 'success=True' in out or 'success=true' in out
        # 手动确认后允许导航（用户对当前红点-地图对齐负责）
        if ok and _web_ops_node is not None:
            _web_ops_node._loc_user_accepted = True
            _web_ops_node._loc_aligned = True
            _web_ops_node._loc_hit = max(_web_ops_node._loc_hit, 0.60)
        return {'ok': ok, 'log': out[-400:], 'error': None if ok else (out[-200:] or 'accept failed')}
    except subprocess.TimeoutExpired:
        return {'ok': False, 'error': 'accept_reloc_pose 超时'}
    except Exception as e:
        return {'ok': False, 'error': str(e)}


def call_relocalize(timeout=35.0):
    """Call /relocalize Trigger service (blocking, for HTTP API)."""
    loc = ensure_localization_stack()
    if not loc.get('fastlio'):
        return {'ok': False, 'error': 'Super-LIO 未运行，无法重定位（先恢复激光里程计）', 'loc': loc}
    ensure_scan_node()
    if not proc_alive('auto_relocalize'):
        if not ensure_auto_relocalize():
            return {'ok': False, 'error': 'auto_relocalize 未运行'}
        time.sleep(1.0)
    try:
        open('/tmp/auto_relocalize.log', 'a').write('\n==== relocalize request ====\n')
    except Exception:
        pass
    script = (
        'source /opt/ros/humble/setup.bash && '
        'source /home/linaro/robot_ws/install/setup.bash && '
        'ros2 service call /relocalize std_srvs/srv/Trigger'
    )
    try:
        r = subprocess.run(
            ['bash', '-lc', script],
            capture_output=True, text=True, timeout=timeout)
        out = (r.stdout or '') + (r.stderr or '')
        called = 'success=True' in out or 'success=true' in out
        aligned = False
        import re
        m = re.search(r'黑点命中=([0-9.]+)%', out)
        hit = float(m.group(1)) / 100.0 if m else None
        if hit is not None:
            aligned = hit >= 0.60
        elif 'aligned' in out and 'weak' not in out and 'mid' not in out:
            aligned = True
        err = None
        if not called:
            err = out[-300:] or 'relocalize failed'
        elif not aligned:
            err = '定位置信度尚未≥60%（将继续监测或请确认位姿）'
        return {
            'ok': bool(called and aligned),
            'called': bool(called),
            'aligned': bool(aligned),
            'hit': hit,
            'log': out[-800:],
            'error': err,
        }
    except subprocess.TimeoutExpired:
        return {'ok': False, 'error': '重定位超时（地图/激光可能未就绪）'}
    except Exception as e:
        return {'ok': False, 'error': str(e)}


def global_relocalize(timeout=20.0):
    """3D-BBS 全局重定位：任意位置触发一次 -> /global_pose -> /initialpose -> reloc 精配准。"""
    if not proc_alive('bbs3d_ros2_node'):
        return {'ok': False, 'error': '全局重定位未启动（先把开机模式切到 greloc，或跑 start_global_reloc.sh）'}
    for name in ('livox_cloud_to_pc2', 'bbs_global_pose_bridge'):
        if not proc_alive(name):
            return {'ok': False, 'error': f'{name} 未运行（start_global_reloc.sh）'}
    trig = (
        'source /opt/ros/humble/setup.bash && '
        'source /home/linaro/robot_ws/install/setup.bash && '
        'ros2 topic pub --once /bbs_localize std_msgs/msg/Bool "{data: true}"'
    )
    try:
        subprocess.run(['bash', '-lc', trig], capture_output=True, text=True, timeout=8.0)
    except Exception as e:
        return {'ok': False, 'error': f'触发 3D-BBS 失败: {e}'}
    score = None
    deadline = time.time() + timeout
    while time.time() < deadline:
        time.sleep(1.0)
        try:
            log = open('/tmp/bbs3d.log').read()[-4000:]
        except Exception:
            log = ''
        m = re.findall(r'Localize: success \(score=(\d+)', log)
        if m:
            score = int(m[-1])
            break
    if score is None:
        return {'ok': False, 'error': '3D-BBS 未成功（查 /tmp/bbs3d.log：地图/点云/IMU 是否就绪）'}
    converged = False
    for _ in range(8):
        time.sleep(1.0)
        try:
            rlog = open('/tmp/reloc_run.log').read()[-6000:]
        except Exception:
            rlog = ''
        if 'Converged Succeed' in rlog:
            converged = True
            break
    if _web_ops_node is not None and converged:
        _web_ops_node._loc_user_accepted = True
        _web_ops_node._loc_aligned = True
        _web_ops_node._loc_hit = max(_web_ops_node._loc_hit, 0.80)
    return {'ok': bool(converged), 'score': score, 'converged': bool(converged),
            'error': None if converged else '3D-BBS 成功但 reloc 未收敛，可再试一次'}


def set_reloc_mode(body):
    """Toggle auto_relocalize's watchdog live via ros2 param set (no nav restart).

    mode='auto' -> watchdog_en:true  (持续监测，定位丢失时自动全局重定位)
    mode='odom' -> watchdog_en:false (侧重里程计，不自动重定位；仍可手动重定位)
    """
    mode = 'odom' if body.get('mode') == 'odom' else 'auto'
    if not proc_alive('auto_relocalize'):
        return {'ok': False, 'error': 'auto_relocalize 未运行', 'mode': mode}
    script = (
        'source /opt/ros/humble/setup.bash && '
        'source /home/linaro/robot_ws/install/setup.bash && '
        'ros2 param set /auto_relocalize watchdog_en %s'
        % ('true' if mode == 'auto' else 'false')
    )
    try:
        r = subprocess.run(['bash', '-lc', script], capture_output=True, text=True, timeout=10)
        out = (r.stdout or '') + (r.stderr or '')
        ok = 'Set parameter successful' in out or r.returncode == 0
        return {'ok': ok, 'mode': mode, 'log': out[-300:] if not ok else None}
    except Exception as e:
        return {'ok': False, 'error': str(e), 'mode': mode}


def reload_nav_map(body):
    """Hot-reload map via map_server and optionally force relocalize."""
    global _nav2_current_map
    m = body.get('map') or body.get('name') or ''
    m = os.path.basename(m)
    if m.endswith('.pgm'):
        m = m[:-4] + '.yaml'
    if not m.endswith('.yaml'):
        m += '.yaml'
    map_path = os.path.join(MAPS_DIR, m)
    if not os.path.isfile(map_path):
        return {'ok': False, 'error': f'map not found: {m}'}
    if not proc_alive('component_container_isolated'):
        return {'ok': False, 'error': 'Nav2 未运行，请先启动导航'}
    _nav2_current_map = map_path
    # nav2_msgs/srv/LoadMap
    script = (
        'source /opt/ros/humble/setup.bash && '
        'source /home/linaro/robot_ws/install/setup.bash && '
        'ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap '
        '"{map_url: \'%s\'}"' % map_path
    )
    try:
        r = subprocess.run(
            ['bash', '-lc', script],
            capture_output=True, text=True, timeout=20)
        out = (r.stdout or '') + (r.stderr or '')
        # result=0 is RESULT_SUCCESS
        ok = 'result=0' in out or 'RESULT_SUCCESS' in out or r.returncode == 0
        reloc = None
        if ok and body.get('reloc', True):
            time.sleep(0.8)
            reloc = call_relocalize(timeout=40.0)
        return {
            'ok': ok, 'map': m, 'log': out[-600:],
            'reloc': reloc,
            'error': None if ok else (out[-300:] or 'load_map failed'),
        }
    except Exception as e:
        return {'ok': False, 'error': str(e)}


def stop_nav2_api():
    global _nav2_current_map
    stop_nav2_procs()
    _nav2_current_map = None
    return {'ok': True}


def start_nav2(body):
    global _nav2_starting
    with _nav2_lock:
        if _nav2_starting:
            return {'ok': False, 'error': '导航正在启动中，请稍候'}
        _nav2_starting = True
    try:
        return _start_nav2_impl(body)
    finally:
        _nav2_starting = False


def _boot_relocalize_after_nav_start(settle_sec=3.5):
    """启动导航后自动全图重定位一次（自检初始位姿），避免默认 (0,0) 乱指。"""
    try:
        time.sleep(max(1.0, float(settle_sec)))
        for _ in range(20):
            if (proc_alive('auto_relocalize')
                    and proc_alive('component_container_isolated')
                    and _scan_proc_alive()):
                break
            time.sleep(0.5)
        # 再等 AMCL 真正订阅 /initialpose
        time.sleep(1.5)
        try:
            open('/tmp/auto_relocalize.log', 'a').write(
                '\n==== boot auto-relocalize after nav start ====\n')
        except Exception:
            pass
        r = call_relocalize(timeout=40.0)
        hit = r.get('hit')
        msg = 'boot reloc ok' if r.get('ok') else ('boot reloc weak/fail: %s' % (r.get('error') or ''))
        if hit is not None:
            msg += ' hit=%.1f%%' % (hit * 100.0)
        try:
            open('/tmp/auto_relocalize.log', 'a').write(msg + '\n')
        except Exception:
            pass
    except Exception as e:
        try:
            open('/tmp/auto_relocalize.log', 'a').write(
                'boot reloc exception: %s\n' % e)
        except Exception:
            pass


def _start_nav2_impl(body):
    m = body.get('map', '') or ''
    m = os.path.basename(m)
    if m.endswith('.pgm'):
        m = m[:-4] + '.yaml'
    if m and not m.endswith('.yaml'):
        m += '.yaml'
    if not m or not os.path.isfile(os.path.join(MAPS_DIR, m)):
        # Prefer selected map; else map_live; else any yaml
        for cand in ('map_live.yaml', 'map.yaml'):
            if os.path.isfile(os.path.join(MAPS_DIR, cand)):
                m = cand
                break
        else:
            yamls = [f for f in os.listdir(MAPS_DIR) if f.endswith('.yaml') and not f.startswith('_')]
            if not yamls:
                return {'ok': False, 'error': '没有可用地图，请先建图保存'}
            m = sorted(yamls)[0]
    map_path = os.path.join(MAPS_DIR, m)
    if not os.path.isfile(map_path):
        return {'ok': False, 'error': f'map not found: {m}'}

    # 幂等: 已经用同一张图跑着就别整套拆了重来 —— 全局重定位是"开盲盒"，
    # 反复点「启动导航」会拿一次刚配准好的定位去赌一次新的盲搜，可能越点越偏。
    global _nav2_current_map
    if (proc_alive('component_container_isolated') and proc_alive('auto_relocalize')
            and _nav2_current_map == map_path):
        ensure_scan_node()
        return {
            'ok': True, 'map': m, 'already_running': True,
            'hint': '导航已经在跑（同一张图）；定位不对请点「重定位」',
            'auto_reloc': False,
        }
    # 建图/导航二选一，且要对称：之前这里是直接静默 kill_pattern('map_building_node')，
     # 建图中点"启动导航"会在用户毫无察觉的情况下把建图预览杀掉（Super-LIO 本身不停，
    # 点云还在攒，但网页上突然看不到预览了）。现在跟 start_mapping_session() 的
    # "导航中拒绝建图"保持对称：建图中一律拒绝启动导航，不再替用户做这个决定。
    if proc_alive('map_building_node'):
        return {'ok': False, 'error': '正在建图中，请先「停止建图」或「保存地图」后再启动导航'}
    _nav2_current_map = map_path

    # 新启导航：清掉上次的「定位已确认」，等自检重定位结果
    if _web_ops_node is not None:
        _web_ops_node._loc_user_accepted = False
        _web_ops_node._loc_aligned = False
        _web_ops_node._loc_hit = 0.0

    # Singleton Nav2 — previous double-launch left two navigate_to_pose servers
    stop_nav2_procs()

    loc = ensure_localization_stack()
    if not loc['fastlio']:
        return {'ok': False, 'error': 'Super-LIO 未能启动，导航需要里程计 TF（odom→base_link）',
                'loc': loc}

    scan_ok = ensure_scan_node()
    # Append to log with separator so we can see each attempt
    try:
        with open('/tmp/nav2.log', 'a') as f:
            f.write('\n==== %s map=%s ====\n' % (
                time.strftime('%H:%M:%S'), m))
    except Exception:
        pass
    cmd_str = (
        'ros2 launch nav2_bringup bringup_launch.py '
        f'params_file:={NAV2_PARAMS} '
        f'map:={map_path} use_sim_time:=False autostart:=True'
    )
    run_bg(cmd_str, '/tmp/nav2.log')
    # Give map_server a moment, then start auto-relocalize
    time.sleep(2.0)
    reloc_ok = ensure_auto_relocalize()
    # 后台催活：若 planner 仍在等 map→odom，AMCL set_initial_pose 会解；
    # 再兜底把 inactive 的 bt/velocity_smoother 拉起来。
    threading.Thread(target=_heal_nav2_lifecycle, args=(18.0,), daemon=True).start()
    # 启动后自动重定位自检（不阻塞 HTTP 返回）
    if reloc_ok:
        threading.Thread(
            target=_boot_relocalize_after_nav_start, args=(3.5,), daemon=True).start()
    return {
        'ok': True, 'map': m, 'scan': scan_ok, 'loc': loc,
        'auto_relocalize': reloc_ok,
        'auto_reloc': bool(reloc_ok),
        'hint': '已启动，正在自动重定位自检…' if reloc_ok
                else '已启动，但重定位节点未起来，请手动点「重定位」',
    }


def _heal_nav2_lifecycle(timeout_sec=18.0):
    """Activate Nav2 lifecycle nodes left inactive after a stuck planner activate.

    Uses `ros2 lifecycle` subprocesses (not in-process rclpy) so we never
    clash with web_ops_node's own rclpy context.
    """
    names = [
        'controller_server', 'smoother_server', 'planner_server',
        'bt_navigator', 'behavior_server', 'waypoint_follower', 'velocity_smoother',
    ]
    deadline = time.time() + float(timeout_sec)
    while time.time() < deadline and not proc_alive('component_container_isolated'):
        time.sleep(0.4)
    if not proc_alive('component_container_isolated'):
        return
    time.sleep(3.0)
    env = dict(os.environ)
    env['ROS_HOME'] = env.get('ROS_HOME', '/tmp/ros_home')
    prefix = (
        'source /opt/ros/humble/setup.bash && '
        'source /home/linaro/robot_ws/install/setup.bash && '
    )
    while time.time() < deadline:
        pending = False
        for name in names:
            try:
                st = subprocess.run(
                    ['bash', '-lc', prefix + f'ros2 lifecycle get /{name}'],
                    capture_output=True, text=True, timeout=4, env=env)
                out = ((st.stdout or '') + (st.stderr or '')).strip().lower()
            except Exception:
                pending = True
                continue
            # NOTE: "inactive" contains substring "active" — check inactive first.
            if 'unconfigured' in out:
                pending = True
                try:
                    subprocess.run(
                        ['bash', '-lc', prefix + f'ros2 lifecycle set /{name} configure'],
                        capture_output=True, text=True, timeout=6, env=env)
                    subprocess.run(
                        ['bash', '-lc', prefix + f'ros2 lifecycle set /{name} activate'],
                        capture_output=True, text=True, timeout=8, env=env)
                except Exception:
                    pass
            elif 'inactive' in out:
                pending = True
                try:
                    subprocess.run(
                        ['bash', '-lc', prefix + f'ros2 lifecycle set /{name} activate'],
                        capture_output=True, text=True, timeout=8, env=env)
                except Exception:
                    pass
            elif 'active' not in out:
                pending = True
        if not pending:
            break
        time.sleep(1.2)


def restart_rosbridge():
    for pid in proc_pids('rosbridge_websocket'):
        try:
            os.kill(pid, signal.SIGKILL)
        except Exception:
            pass
    time.sleep(2)
    run_bg('ros2 launch rosbridge_server rosbridge_websocket_launch.xml', '/tmp/rosbridge.log')
    return {'ok': True}


def publish_cmd_vel(body):
    """网页摇杆主通路：HTTP → web_ops → /cmd_vel。

    带单调 seq：高延迟 WiFi 下晚到的旧非零包不得覆盖更新的零速，
    否则会出现「松手还走」。零速指令始终接受。
    无 seq 的旧前端：零速后 1.5s 内拒绝其非零，避免未刷新的标签页抢控制。
    """
    global _cmd_vel_seq, _cmd_vel_stop_mono
    n = _web_ops_node
    if n is None or getattr(n, 'pub_cmd', None) is None:
        return {'ok': False, 'error': 'web_ops not ready'}
    try:
        vx = float(body.get('vx', 0.0) or 0.0)
        vy = float(body.get('vy', 0.0) or 0.0)
        wz = float(body.get('wz', 0.0) or 0.0)
        try:
            seq = int(body.get('seq', 0) or 0)
        except (TypeError, ValueError):
            seq = 0
        is_stop = abs(vx) < 1e-9 and abs(vy) < 1e-9 and abs(wz) < 1e-9
        now = time.monotonic()
        with _cmd_vel_lock:
            if not is_stop:
                if seq > 0:
                    if seq < _cmd_vel_seq:
                        return {'ok': True, 'ignored': 'stale'}
                    _cmd_vel_seq = seq
                else:
                    # 旧前端无 seq：若刚下过零速，丢掉以免「松手还走」
                    if (now - _cmd_vel_stop_mono) < 1.5:
                        return {'ok': True, 'ignored': 'post_stop'}
                    if _cmd_vel_seq > 0:
                        return {'ok': True, 'ignored': 'need_seq'}
            else:
                if seq >= _cmd_vel_seq:
                    _cmd_vel_seq = seq
                _cmd_vel_stop_mono = now
            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = vy
            msg.angular.z = wz
            n.pub_cmd.publish(msg)
            if is_stop:
                for _ in range(2):
                    n.pub_cmd.publish(msg)
        return {'ok': True, 'seq': _cmd_vel_seq, 'stop': is_stop}
    except Exception as e:
        return {'ok': False, 'error': str(e)}


def set_ctrl_mode(mode):
    """APP = 释放 SDK 给遥控器；SDK = 重新拉起 genisom_bridge。"""
    mode = (mode or '').strip().upper()
    if mode not in ('APP', 'SDK'):
        return {'ok': False, 'error': 'mode must be APP or SDK'}

    def bridge_pids():
        out = []
        for pid in proc_pids('genisom_bridge'):
            try:
                args = subprocess.run(
                    ['ps', '-p', str(pid), '-o', 'args='],
                    capture_output=True, text=True, timeout=2).stdout or ''
            except Exception:
                args = ''
            if 'web_ops' in args or 'web_ui' in args:
                continue
            if 'genisom_bridge' in args:
                out.append(pid)
        return out

    if mode == 'APP':
        kill_pattern('ros2 launch genisom_bridge')
        for pid in bridge_pids():
            try:
                os.kill(pid, signal.SIGTERM)
            except Exception:
                pass
        time.sleep(0.6)
        for pid in bridge_pids():
            try:
                os.kill(pid, signal.SIGKILL)
            except Exception:
                pass
        time.sleep(0.3)
        alive = bool(bridge_pids())
        return {'ok': not alive, 'ctrl': 'SDK' if alive else 'APP',
                'msg': '已切换到 APP（遥控器可用）' if not alive else '未能停止 SDK bridge'}

    if not bridge_pids():
        run_bg('ros2 launch genisom_bridge bridge.launch.py', '/tmp/bridge_run.log')
        time.sleep(1.2)
    alive = bool(bridge_pids())
    return {'ok': alive, 'ctrl': 'SDK' if alive else 'APP',
            'msg': '已切换到 SDK' if alive else '启动 genisom_bridge 失败'}


def tail_log(file, n=50):
    safe = os.path.basename(file)
    p = '/tmp/' + safe
    if not os.path.isfile(p):
        p = '/home/linaro/robot_ws/' + safe
    if not os.path.isfile(p):
        return 'log not found: ' + safe
    try:
        r = subprocess.run(['tail', '-n', str(n), p], capture_output=True, text=True, timeout=5)
        return r.stdout[-8000:]
    except Exception as e:
        return 'err: ' + str(e)


def service_status():
    return {
        'livox': proc_alive('livox_ros_driver2'),
        'bridge': proc_alive('genisom_bridge'),
        'fastlio': proc_alive('super_lio_node'),
        'lio_tf': proc_alive('lio_tf_bridge'),
        'map_building': proc_alive('map_building_node'),
        'scan': _scan_proc_alive(),
        'nav2': proc_alive('component_container_isolated'),
        'auto_relocalize': proc_alive('auto_relocalize'),
        'rosbridge': proc_alive('rosbridge_websocket'),
        # 本进程自己就是 web_ops；proc_alive 会跳过 me，这里直接 True
        'webops': True,
    }


def battery_json():
    """电量兜底：后端缓存的 /battery_state，走 HTTP 给前端（不依赖 rosbridge 订阅）。"""
    n = _web_ops_node
    b = getattr(n, '_battery', None) if n is not None else None
    age = None
    if n is not None:
        ts = getattr(n, '_battery_ts', 0.0) or 0.0
        if ts:
            age = round(time.monotonic() - ts, 1)
    if b is None:
        return {'ok': False, 'percentage': None, 'age_sec': age, 'sdk': False}
    try:
        return {
            'ok': True,
            'percentage': float(b.percentage),
            'voltage': float(b.voltage),
            'age_sec': age,
            'sdk': age is not None and age < 5.0,
        }
    except Exception:
        return {'ok': False, 'percentage': None, 'age_sec': age, 'sdk': False}


_sdk_heal_ts = 0.0


def heal_sdk_bridge():
    """狗 SDK 掉线时电量/遥控全挂：进程还在但无 /battery_state → 重启 bridge。"""
    global _sdk_heal_ts
    n = _web_ops_node
    if n is None:
        return
    # APP 模式故意无 bridge
    if not proc_alive('genisom_bridge'):
        return
    ts = getattr(n, '_battery_ts', 0.0) or 0.0
    # 启动宽限 25s；之后超过 12s 无电量则重启
    up = time.monotonic() - getattr(n, '_boot_mono', time.monotonic())
    if up < 25.0:
        return
    age = (time.monotonic() - ts) if ts else up
    if age < 12.0:
        return
    now = time.monotonic()
    if (now - _sdk_heal_ts) < 45.0:
        return
    _sdk_heal_ts = now
    try:
        n.get_logger().warn(
            'SDK/battery stale (%.1fs) — restarting genisom_bridge' % age)
    except Exception:
        pass
    # 复用 SDK 模式拉起（会先杀再启）
    try:
        set_ctrl_mode('SDK')
    except Exception:
        kill_pattern('genisom_bridge')
        time.sleep(1.0)
        run_bg('ros2 launch genisom_bridge bridge.launch.py', '/tmp/bridge_run.log')


def _cpu_pct():
    """Sample /proc/stat twice for approximate CPU usage percent."""
    def read():
        with open('/proc/stat') as f:
            parts = f.readline().split()
        vals = [int(x) for x in parts[1:]]
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
        return idle, sum(vals)
    try:
        i1, t1 = read()
        time.sleep(0.12)
        i2, t2 = read()
        dt, di = t2 - t1, i2 - i1
        if dt <= 0:
            return 0.0
        return round(max(0.0, min(100.0, (1.0 - di / dt) * 100.0)), 1)
    except Exception:
        return None


def _fmt_uptime(sec):
    sec = int(sec)
    d, sec = divmod(sec, 86400)
    h, sec = divmod(sec, 3600)
    m, s = divmod(sec, 60)
    if d > 0:
        return f'{d}天 {h:02d}:{m:02d}:{s:02d}'
    return f'{h:02d}:{m:02d}:{s:02d}'


def sysinfo():
    info = {}
    try:
        with open('/proc/loadavg') as f:
            info['load'] = f.read().strip()
        with open('/proc/uptime') as f:
            up_sec = float(f.read().split()[0])
            info['uptime_sec'] = int(up_sec)
            info['uptime'] = _fmt_uptime(up_sec)
        info['cpu_pct'] = _cpu_pct()
        with open('/proc/meminfo') as f:
            total = avail = 0
            for line in f:
                if line.startswith('MemTotal'):
                    total = int(line.split()[1])
                elif line.startswith('MemAvailable'):
                    avail = int(line.split()[1])
            used = max(0, total - avail)
            info['mem'] = f'{avail // 1024}MB / {total // 1024}MB 可用'
            info['mem_pct'] = round(used * 100.0 / total, 1) if total else None
            info['mem_used_mb'] = used // 1024
            info['mem_total_mb'] = total // 1024
        r = subprocess.run(['df', '-B1', '/'], capture_output=True, text=True, timeout=5)
        # Filesystem 1B-blocks Used Available Use% Mounted
        parts = r.stdout.splitlines()[1].split()
        disk_total, disk_used = int(parts[1]), int(parts[2])
        info['disk'] = f'{int(parts[3]) // (1024 ** 3)}GB 可用'
        info['disk_pct'] = round(disk_used * 100.0 / disk_total, 1) if disk_total else None
        info['services'] = service_status()
        # SDK bridge alive => SDK 通道；否则视为遥控器/APP
        info['ctrl'] = 'SDK' if info['services'].get('bridge') else 'APP'
        r = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=5)
        info['net'] = r.stdout.strip()
        # Prefer wlan0 for UI hint
        try:
            w = subprocess.run(
                ['bash', '-c', "ip -4 -o addr show dev wlan0 | awk '{print $4}' | cut -d/ -f1"],
                capture_output=True, text=True, timeout=3)
            info['wlan_ip'] = (w.stdout or '').strip()
        except Exception:
            info['wlan_ip'] = ''
        info['video_mjpeg'] = '/api/video/mjpeg'
        info['video_rtsp'] = DOG_RTSP
        info['video_width'] = int((_video_cfg or {}).get('width', VIDEO_WIDTH))
        info['video_fps'] = int((_video_cfg or {}).get('fps', VIDEO_FPS))
        info['video_q'] = int((_video_cfg or {}).get('q', VIDEO_Q))
        info['video_profile'] = _video_profile
        info['video_profile_label'] = VIDEO_PROFILES.get(_video_profile, {}).get('label', '')
    except Exception as e:
        info['error'] = str(e)
    return info


# ---------------- ROS 节点 ----------------
class WebOpsNode(Node):
    def __init__(self):
        super().__init__('web_ops_node')
        self.sub_nav_cmd = self.create_subscription(Twist, '/web/nav_cmd', self.cb_nav_cmd, 10)
        self.sub_nav_cancel = self.create_subscription(Empty, '/web/nav_cancel', self.cb_nav_cancel, 10)
        self.sub_init_pose = self.create_subscription(Twist, '/web/init_pose', self.cb_init_pose, 10)
        self.sub_start_nav2 = self.create_subscription(Empty, '/web/start_nav2', self.cb_start_nav2, 10)
        self.sub_stop_nav2 = self.create_subscription(Empty, '/web/stop_nav2', self.cb_stop_nav2, 10)
        self.sub_mapping = self.create_subscription(Empty, '/web/mapping', self.cb_mapping, 10)

        # 电量：后端缓存一份，走 /api/battery 让前端 HTTP 拉，不依赖 rosbridge 订阅
        self._battery = None
        self._battery_ts = 0.0
        self._boot_mono = time.monotonic()
        self.create_subscription(BatteryState, '/battery_state', self.cb_battery, 10)

        # TRANSIENT_LOCAL：重定位节点晚订阅也能拿到当前导航状态，避免导航中误触发重搜
        _nav_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.pub_nav_status = self.create_publisher(String, '/web/nav_status', _nav_qos)
        self.pub_sys_status = self.create_publisher(String, '/web/sys_status', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # 网页 TFClient 常拿不到 map→base_link；这里查 TF 后转发给前端画激光
        self.pub_robot_pose = self.create_publisher(PoseStamped, '/web/robot_pose', 10)
        # 把 /scan 按「激光时间戳」转到 map，前端直接画 map 系红点，避免箭头位姿与激光不同步导致一走就漂
        self.pub_scan_map = self.create_publisher(Float32MultiArray, '/web/scan_map', 10)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(
            LaserScan, '/scan', self.cb_scan_to_map, qos_profile_sensor_data)
        # /map 是 map_server 一次性 TRANSIENT_LOCAL 发布的；rosbridge 对任意话题固定
        # 用 BEST_EFFORT/VOLATILE 订阅（这里这版 roslib.min.js 也不支持传 QoS 覆盖），
        # 只要浏览器这边的 rosbridge 订阅是在那一次性发布*之后*才建立的——WiFi 掉线
        # 重连、或者页面比 Nav2 启动得晚——就永远收不到，网页上"导航加载不出地图"。
        # 用正确匹配的 QoS 在后端订阅一次缓存下来，配 /api/nav2/map 走 HTTP 一次性
        # 拉取，不依赖 rosbridge 的订阅时序。
        self._last_map = None
        self.create_subscription(
            OccupancyGrid, '/map', self.cb_map,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1))

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_goal_handle = None
        self._nav_goal_token = 0  # 递增；旧 goal 的 cancel/result 回调不得覆盖新状态
        self.nav_state = 'idle'
        # 未对齐禁止下发 Nav2 目标（AMCL set_initial_pose=(0,0) 只为起 lifecycle，不是真定位）
        self._loc_aligned = False
        # cb_status_timer 用：heal_nav_deps() 挪到后台线程跑，这把锁保证同一时刻只有一条在跑
        self._heal_lock = threading.Lock()
        self._loc_user_accepted = False  # 用户点过「接受位姿」/手动设姿
        self._loc_hit = 0.0
        self.sub_reloc_status = self.create_subscription(
            String, '/relocalize_status', self.cb_reloc_status, 10)

        self.create_timer(1.0, self.cb_status_timer)
        self.create_timer(0.05, self.cb_robot_pose_timer)
        self._set_nav_status('idle', '')

    def cb_reloc_status(self, msg):
        s = (msg.data or '').strip()
        hit = 0.0
        if ':' in s:
            try:
                hit = float(s.split(':', 1)[1])
            except Exception:
                hit = 0.0
        self._loc_hit = hit
        base = s.split(':', 1)[0]
        # 重新盲搜 / 明确要求手动 → 清掉用户确认
        if base in ('searching', 'waiting_amcl', 'need_manual', 'need_init', 'failed', 'critical'):
            self._loc_user_accepted = False
        # 允许导航：命中≥60% 的 aligned/hold，或用户已确认当前位姿
        if base in ('aligned', 'hold') and hit >= 0.59:
            self._loc_aligned = True
        elif self._loc_user_accepted:
            self._loc_aligned = True
        else:
            self._loc_aligned = False

    def cb_map(self, msg: OccupancyGrid):
        self._last_map = msg

    def cb_scan_to_map(self, scan: LaserScan):
        """Project LaserScan into map frame at the scan stamp (not latest pose)."""
        if not proc_alive('component_container_isolated'):
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', scan.header.frame_id,
                Time.from_msg(scan.header.stamp),
                timeout=Duration(seconds=0.05))
        except Exception:
            try:
                tf = self._tf_buffer.lookup_transform(
                    'map', scan.header.frame_id, Time(),
                    timeout=Duration(seconds=0.02))
            except Exception:
                return
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        bx = tf.transform.translation.x
        by = tf.transform.translation.y
        pts = []
        a = scan.angle_min
        rmin = scan.range_min or 0.05
        rmax = scan.range_max or 30.0
        for r in scan.ranges:
            if math.isfinite(r) and rmin < r < rmax:
                lx = r * math.cos(a)
                ly = r * math.sin(a)
                pts.append(bx + lx * c - ly * s)
                pts.append(by + lx * s + ly * c)
            a += scan.angle_increment
        if len(pts) < 8:
            return
        msg = Float32MultiArray()
        msg.data = pts
        self.pub_scan_map.publish(msg)

    def cb_robot_pose_timer(self):
        """Republish map→base_footprint for the web console (scan overlay + dog arrow).

        /scan is published in base_footprint (nav_scan_node); fall back to base_link.
        """
        if not proc_alive('component_container_isolated'):
            return
        t = None
        for frame in ('base_footprint', 'base_link'):
            try:
                t = self._tf_buffer.lookup_transform(
                    'map', frame, Time(),
                    timeout=Duration(seconds=0.05))
                break
            except Exception:
                continue
        if t is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.position.z = 0.0
        # 只用平面航向，避免 roll/pitch 渗进网页 yaw（箭头左右偏）
        q = t.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # 与 lio_tf_bridge 一致：用机体系 +X 投影，俯仰时更稳
        # R*[1,0,0]
        fx = 1 - 2 * (q.y * q.y + q.z * q.z)
        fy = 2 * (q.x * q.y + q.z * q.w)
        if fx * fx + fy * fy > 1e-8:
            yaw = math.atan2(fy, fx)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        self.pub_robot_pose.publish(msg)

    def cb_nav_cmd(self, msg):
        x, y, yaw = msg.linear.x, msg.linear.y, msg.angular.z
        self.get_logger().info(f'nav_goal ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f}deg)')
        if abs(x) < 0.02 and abs(y) < 0.02:
            return
        # 网页/rosbridge 偶发连发同一目标 2～3 次 → cancel/preempt 风暴，加重假到达
        now = time.time()
        last = getattr(self, '_last_nav_cmd', None)
        if last and (now - last[0]) < 0.6 and abs(last[1] - x) < 0.05 and abs(last[2] - y) < 0.05:
            self.get_logger().info('nav_goal debounced (duplicate)')
            return
        self._last_nav_cmd = (now, x, y, yaw)
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self._set_nav_status('aborted', 'nav2_server_unavailable')
            return
        # 先取消旧目标；用 token 忽略旧 result，避免把新导航刷成 canceled
        self._nav_goal_token += 1
        send_token = self._nav_goal_token
        old = self.nav_goal_handle
        self.nav_goal_handle = None
        if old is not None:
            try:
                old.cancel_goal_async()
            except Exception:
                pass
            time.sleep(0.25)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        f = self.nav_client.send_goal_async(goal)
        f.add_done_callback(lambda fut, tok=send_token: self._goal_sent_cb(fut, tok))

    def _goal_sent_cb(self, future, token):
        if token != self._nav_goal_token:
            return
        try:
            h = future.result()
        except Exception as e:
            self._set_nav_status('aborted', f'goal_send_{e}')
            return
        if h is None or not h.accepted:
            self._set_nav_status('aborted', 'goal_rejected')
            return
        self.nav_goal_handle = h
        self._set_nav_status('navigating', '')
        h.get_result_async().add_done_callback(
            lambda fut, tok=token, handle=h: self._goal_result_cb(fut, tok, handle))

    def _goal_result_cb(self, future, token, handle):
        # 已被更新的目标 / 替换发送：丢弃过期回调
        if token != self._nav_goal_token or self.nav_goal_handle is not handle:
            return
        try:
            st = future.result().status
        except Exception as e:
            self._set_nav_status('aborted', f'result_{e}')
            self.nav_goal_handle = None
            return
        # GoalStatus: SUCCEEDED=4, CANCELED=5, ABORTED=6 (action_msgs)
        if st == 4:
            self._set_nav_status('reached', '')
        elif st == 5:
            self._set_nav_status('canceled', '')
        else:
            self._set_nav_status('aborted', f'status_{st}')
        self.nav_goal_handle = None

    def cb_nav_cancel(self, msg):
        self._nav_goal_token += 1
        if self.nav_goal_handle is not None:
            try:
                self.nav_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.nav_goal_handle = None
            self._set_nav_status('canceled', '')
        else:
            self._set_nav_status('canceled', 'no_active_goal')
        self._publish_zero()

    def _publish_zero(self):
        try:
            for _ in range(5):
                self.pub_cmd.publish(Twist())
                time.sleep(0.05)
        except Exception:
            pass

    def cb_battery(self, msg):
        self._battery = msg
        self._battery_ts = time.monotonic()

    def cb_init_pose(self, msg):
        x, y, yaw = msg.linear.x, msg.linear.y, msg.angular.z
        self.get_logger().info(f'init_pose ({x:.2f}, {y:.2f}, yaw {math.degrees(yaw):.0f}deg)')
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        # stamp=0 → AMCL 用最新 TF
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0
        pose.pose.pose.position.x = float(x)
        pose.pose.pose.position.y = float(y)
        pose.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.25
        self.pub_init.publish(pose)
        # 用户拖了初始位姿 = 认可当前对齐，允许下目标
        self._loc_user_accepted = True
        self._loc_aligned = True
        self._loc_hit = max(self._loc_hit, 0.60)

    def cb_mapping(self, msg):
        set_mapping_session(True)
        if not proc_alive('super_lio_node'):
            run_bg(f'bash {START_ALL} mapping', '/tmp/mapping.log')
            self.get_logger().info('mapping chain restarted via start_all.sh mapping')
        else:
            # Ensure map_building preview is up
            if not proc_alive('map_building_node'):
                run_bg(
                    'ros2 launch nav2_tools map_building.launch.py',
                    '/tmp/map_building.log')

    def cb_start_nav2(self, msg):
        self.get_logger().info('start nav2 (ROS trigger)')
        # Prefer live map; start_nav2 resolves missing names itself
        start_nav2({'map': 'map_live.yaml'})
        # 启导航时清掉上次卡死的 navigating（无 goal handle）
        self.nav_goal_handle = None
        self._nav_goal_token += 1
        self._set_nav_status('idle', 'nav2_started')

    def cb_stop_nav2(self, msg):
        stop_nav2_api()
        self._publish_zero()
        self.nav_goal_handle = None
        self._nav_goal_token += 1
        self._set_nav_status('idle', 'nav2_stopped')

    def cb_status_timer(self):
        # heal_nav_deps() 内部（ensure_localization_stack/ensure_scan_node/
        # ensure_auto_relocalize）本来就是设计成阻塞轮询、最长能堵 10+ 秒——
        # 这些函数也被 /api/relocalize、/api/nav2/start 等 HTTP 接口直接同步
        # 调用，那些地方就是要等到准确结果，不能改成非阻塞。
        # 但这里是 1Hz ROS 定时器回调，rclpy 默认单线程 executor：直接同步调用
        # heal_nav_deps() 会把这条回调，连带 /web/sys_status 的周期发布和
        # cb_nav_cmd（导航目标）等其它订阅回调，一起卡住最长 10+ 秒。
        # 2026-09-28 实测：super_lio_node 内存涨到 4GB+ 被 OOM 杀掉后，
        # heal_nav_deps 反复触发这段阻塞重启轮询，其间 /web/nav_cmd 收不到——
        # 这就是「导航页设了目标机器人不走」的根因之一。
        # 挪到后台线程跑，_heal_lock 保证同一时刻只有一条在跑，不重叠。
        if self._heal_lock.acquire(blocking=False):
            def _heal():
                try:
                    heal_nav_deps()
                except Exception:
                    pass
                try:
                    heal_sdk_bridge()
                except Exception:
                    pass
                try:
                    if video_stack_busy() and _video_profile != 'smooth':
                        set_video_profile('smooth')
                except Exception:
                    pass
                finally:
                    self._heal_lock.release()
            threading.Thread(target=_heal, daemon=True).start()
        # Nav2 挂了 / 无活动 goal 时不要一直刷 navigating，否则网页以为还在导
        if self.nav_state == 'navigating' and self.nav_goal_handle is None:
            if not self.nav_client.server_is_ready():
                self._set_nav_status('idle', 'nav2_gone')
        # 到达/取消/失败是瞬时事件：过几秒清回 idle，避免 1Hz 重发 + TRANSIENT_LOCAL
        # 让网页反复弹「已到达目标」
        if self.nav_state in ('reached', 'aborted', 'canceled'):
            age = time.time() - getattr(self, '_nav_state_ts', 0)
            if age > 2.5:
                self._set_nav_status('idle', '')
        status = service_status()
        status['nav'] = self.nav_state
        self.pub_sys_status.publish(String(data=json.dumps(status)))
        # 周期性重发导航状态（配合 TRANSIENT_LOCAL），防止 reloc 漏订阅
        self._set_nav_status(self.nav_state, getattr(self, '_nav_detail', '') or '')

    def _set_nav_status(self, state, detail):
        prev = getattr(self, 'nav_state', None)
        self.nav_state = state
        self._nav_detail = detail or ''
        if prev != state:
            self._nav_state_ts = time.time()
        # 同状态同 detail 的周期重发：只刷新 latch，内容不变
        self.pub_nav_status.publish(
            String(data=state if not detail else f'{state}:{detail}'))


def main(args=None):
    global _web_ops_node
    rclpy.init(args=args)
    node = WebOpsNode()
    _web_ops_node = node
    # 启动时先清掉叠出来的重负载孤儿
    try:
        stop_video_feeder()
        collapse_map_building(keep_one=True)
    except Exception:
        pass
    server = ThreadingHTTPServer(('0.0.0.0', 8090), ApiHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    threading.Thread(target=video_idle_reaper, daemon=True).start()
    node.get_logger().info('HTTP API listening on 8090')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # ExternalShutdownException when parent kills the process — not a real fault
        if type(e).__name__ != 'ExternalShutdownException':
            node.get_logger().error('spin stopped: %s' % e)
    finally:
        try:
            stop_video_feeder()
        except Exception:
            pass
        try:
            server.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
