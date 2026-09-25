#!/usr/bin/env python3
"""web_ops_node - 网页上位机后端代理节点（v3：ROS + 管理 HTTP API）

ROS 订阅(前端 -> 主控):
  /web/nav_cmd     geometry_msgs/Twist   linear.x/y=目标 angular.z=yaw(rad) -> Nav2
  /web/nav_cancel  std_msgs/Empty        取消当前导航目标
  /web/init_pose   geometry_msgs/Twist   linear.x=x linear.y=y angular.z=yaw(rad) -> /initialpose
  /web/start_nav2  std_msgs/Empty        启动 Nav2（默认 map.yaml）
  /web/stop_nav2   std_msgs/Empty        停止 Nav2
  /web/mapping     std_msgs/Empty        确保 FAST-LIO2 建图链运行

ROS 发布(主控 -> 前端):
  /web/nav_status  std_msgs/String  idle|navigating|reached|aborted|canceled[:detail]
  /web/sys_status  std_msgs/String  JSON services + nav
  /web/robot_pose  geometry_msgs/PoseStamped  map→base_link（网页画激光/箭头用；不依赖 TFClient）

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
from geometry_msgs.msg import Twist, Quaternion, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String, Empty
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time

ROS_SETUP = '/opt/ros/humble/setup.bash'
WS_SETUP = '/home/linaro/robot_ws/install/setup.bash'
MAPS_DIR = '/home/linaro/robot_ws/maps'
FASTLIO_PCD = '/home/linaro/robot_ws/src/FAST_LIO/PCD/scans.pcd'
PCD2PGM = '/home/linaro/robot_ws/src/nav2_tools/nav2_tools/pcd2pgm.py'
if not os.path.isfile(PCD2PGM):
    PCD2PGM = '/home/linaro/robot_ws/src/nav2_tools/pcd2pgm.py'
NAV2_PARAMS = '/home/linaro/robot_ws/src/nav2_tools/nav2_params.yaml'
START_ALL = '/home/linaro/robot_ws/start_all.sh'
START_SCAN = '/home/linaro/robot_ws/start_scan_node.sh'
# 狗身广角相机（AgiBot 文档）：有线网段 RTSP H.264
DOG_RTSP = os.environ.get('DOG_RTSP', 'rtsp://192.168.168.168:8554/test')
DOG_FRAME_JPG = '/tmp/dog_live.jpg'
# 画质/帧率：宽边像素、目标帧率、JPEG 质量(2最好~31最差)
VIDEO_WIDTH = int(os.environ.get('DOG_VIDEO_WIDTH', '1280'))
VIDEO_FPS = int(os.environ.get('DOG_VIDEO_FPS', '18'))
VIDEO_Q = int(os.environ.get('DOG_VIDEO_Q', '3'))
_video_feeder = {'proc': None, 'lock': threading.Lock()}


def ensure_video_feeder():
    """Keep one ffmpeg writing latest JPEG for fast snapshot / polling."""
    with _video_feeder['lock']:
        proc = _video_feeder['proc']
        if proc is not None and proc.poll() is None:
            return True
        # 清理残留拉流进程，保证只留一个
        for pid in proc_pids('dog_live.jpg'):
            try:
                os.kill(pid, signal.SIGTERM)
            except Exception:
                pass
        time.sleep(0.15)
        try:
            if proc is not None:
                os.killpg(proc.pid, signal.SIGTERM)
        except Exception:
            pass
        try:
            if os.path.isfile(DOG_FRAME_JPG):
                os.remove(DOG_FRAME_JPG)
        except Exception:
            pass
        w = max(640, min(1920, VIDEO_WIDTH))
        fps = max(8, min(30, VIDEO_FPS))
        q = max(2, min(12, VIDEO_Q))
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
            return True
        except Exception:
            _video_feeder['proc'] = None
            return False


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
            if exe.endswith('python3') or exe.endswith('python'):
                # only if argv looks like the node, not a random script quoting the name
                parts = cmd.split()
                if any(p.endswith(pattern) or p.rstrip('/').endswith(pattern) for p in parts):
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
            elif path == '/api/mapping/status':
                self._send_json(mapping_status())
            elif path == '/api/lidar_extrinsic':
                self._send_json(load_lidar_extrinsic())
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
            elif path == '/api/nav2/reload_map':
                self._send_json(reload_nav_map(body))
            elif path == '/api/restart_rosbridge':
                self._send_json(restart_rosbridge())
            elif path == '/api/ctrl':
                self._send_json(set_ctrl_mode(body.get('mode', '')))
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
            interval = max(0.04, 1.0 / max(8, min(30, VIDEO_FPS)))
            while idle < 100:
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
        data = read_live_frame(wait_sec=2.5)
        if not data:
            cmd = [
                'ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error',
                '-rtsp_transport', 'tcp', '-fflags', 'nobuffer', '-flags', 'low_delay',
                '-y', '-i', rtsp_url or DOG_RTSP,
                '-frames:v', '1', '-q:v', str(max(2, min(12, VIDEO_Q))),
                '-vf', f'scale={max(640, min(1920, VIDEO_WIDTH))}:-2',
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
    removed = []
    for suffix in ('.pgm', '.yaml'):
        p = os.path.join(MAPS_DIR, name[:-4] + suffix)
        if os.path.isfile(p):
            os.remove(p)
            removed.append(suffix)
    return {'ok': True, 'removed': removed}


def stop_fastlio(sig=signal.SIGINT):
    pids = proc_pids('fastlio_mapping')
    for pid in pids:
        try:
            os.kill(pid, sig)
        except Exception:
            pass
    return pids


PCD_DIR = os.path.dirname(FASTLIO_PCD)
LIDAR_EXTRINSIC = '/home/linaro/robot_ws/config/lidar_extrinsic.yaml'


def load_lidar_extrinsic():
    """Read pitch_down / roll / lidar_z for UI + pcd2pgm."""
    out = {
        'ok': True,
        'pitch_down': 0.7853981634,  # dog-head Mid360 looking down ~45°
        'roll': 0.0,
        'lidar_z': 0.45,
        'lidar_pitch': -0.7853981634,
        'mount_pitch_down': 0.7853981634,
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
    """删除 FAST-LIO 上次留下的 PCD，避免旧云污染新图。"""
    removed = []
    try:
        if os.path.isdir(PCD_DIR):
            for fn in os.listdir(PCD_DIR):
                if fn.startswith('scans') and fn.endswith('.pcd'):
                    p = os.path.join(PCD_DIR, fn)
                    try:
                        os.remove(p)
                        removed.append(fn)
                    except Exception:
                        pass
    except Exception as e:
        return {'ok': False, 'error': str(e), 'removed': removed}
    return {'ok': True, 'removed': removed}


def pcd_ready():
    """是否有可转换的 scans.pcd（体积足够）。"""
    try:
        return os.path.isfile(FASTLIO_PCD) and os.path.getsize(FASTLIO_PCD) > 1024
    except Exception:
        return False


def mapping_status():
    running = proc_alive('fastlio_mapping')
    ready = pcd_ready()
    size = 0
    mtime = None
    if os.path.isfile(FASTLIO_PCD):
        try:
            size = os.path.getsize(FASTLIO_PCD)
            mtime = time.strftime('%H:%M:%S', time.localtime(os.path.getmtime(FASTLIO_PCD)))
        except Exception:
            pass
    return {
        'ok': True,
        'fastlio': running,
        'map_building': proc_alive('map_building_node'),
        'pcd_ready': (not running) and ready,
        'pcd_exists': ready,
        'pcd_size': size,
        'pcd_mtime': mtime,
        'can_start': not running and not proc_alive('component_container_isolated'),
        'can_stop': running,
        'can_save': (not running) and ready,
    }


def stop_mapping_keep_pcd():
    """停止 FAST-LIO：先调 /map_save 落盘，再 SIGINT，等待 scans.pcd。"""
    pids = proc_pids('fastlio_mapping')
    if not pids:
        return {
            'ok': True, 'already_stopped': True,
            'pcd_ready': pcd_ready(),
            'pcd_size': os.path.getsize(FASTLIO_PCD) if os.path.isfile(FASTLIO_PCD) else 0,
        }
    # Ask node to flush PCD while still alive
    try:
        subprocess.run(
            ['bash', '-lc',
             'source /opt/ros/humble/setup.bash; source /home/linaro/robot_ws/install/setup.bash; '
             'export ROS_HOME=/tmp/ros_home; '
             'ros2 service call /map_save std_srvs/srv/Trigger "{}"'],
            capture_output=True, text=True, timeout=20)
    except Exception as e:
        print('[stop_mapping] map_save call:', e)

    prev_mtime = os.path.getmtime(FASTLIO_PCD) if os.path.isfile(FASTLIO_PCD) else 0
    stop_fastlio(signal.SIGINT)
    saved = False
    for _ in range(30):
        if os.path.isfile(FASTLIO_PCD):
            mt = os.path.getmtime(FASTLIO_PCD)
            if mt > prev_mtime or (time.time() - mt) < 60:
                time.sleep(0.5)
                if os.path.getsize(FASTLIO_PCD) > 1024:
                    saved = True
                    break
        time.sleep(0.5)
    kill_pattern('map_building_node')
    still = proc_pids('fastlio_mapping')
    # Only force-kill if PCD already on disk; otherwise give a bit more time
    if still and not saved:
        time.sleep(2.0)
        if os.path.isfile(FASTLIO_PCD) and os.path.getsize(FASTLIO_PCD) > 1024:
            saved = True
    for pid in still:
        try:
            os.kill(pid, signal.SIGKILL)
        except Exception:
            pass
    return {
        'ok': True,
        'pcd_ready': pcd_ready(),
        'pcd_saved': saved or pcd_ready(),
        'pcd_size': os.path.getsize(FASTLIO_PCD) if os.path.isfile(FASTLIO_PCD) else 0,
        'force_killed': bool(still),
        'error': None if (saved or pcd_ready()) else '未生成 scans.pcd，请重新建图后再停止（需已编译含落盘的 FAST-LIO）',
    }


def start_mapping_session(body=None):
    """清理旧 PCD 后启动/确保建图链（幂等，不会叠多个 Livox）。"""
    if proc_alive('component_container_isolated'):
        return {'ok': False, 'error': '请先关闭导航'}
    # 若已在建图，仅确保预览
    if proc_alive('fastlio_mapping'):
        if not proc_alive('map_building_node'):
            run_bg(
                'ros2 launch nav2_tools map_building.launch.py',
                '/tmp/map_building.log')
        return {'ok': True, 'already_running': True, 'cleared': []}
    cleared = clear_pcd_cache()
    # start_all.sh is idempotent: collapses duplicate Livox/LIO first
    run_bg(f'bash {START_ALL} mapping', '/tmp/mapping.log')
    return {'ok': True, 'started': True, 'cleared': cleared.get('removed', [])}


def discard_mapping():
    """Stop FAST-LIO without converting PCD to a map."""
    pids = stop_fastlio(signal.SIGINT)
    # Also stop map_building preview node so next session starts clean
    kill_pattern('map_building_node')
    time.sleep(0.5)
    if pids:
        # Force if still alive
        still = proc_pids('fastlio_mapping')
        for pid in still:
            try:
                os.kill(pid, signal.SIGKILL)
            except Exception:
                pass
    return {'ok': True, 'stopped': len(pids), 'discarded': True}


def save_mapping(body):
    """保存建图：SIGINT fastlio -> 等 PCD -> pcd2pgm -> maps/<name>.pgm+yaml"""
    name = os.path.basename(body.get('name', 'map'))
    if name in ('__discard__', 'discard', ''):
        return discard_mapping()
    if name.endswith(('.pgm', '.yaml')):
        name = name.rsplit('.', 1)[0]
    name = re.sub(r'[^\w\-]+', '_', name) or 'map'

    pids = proc_pids('fastlio_mapping')
    if not pids:
        if os.path.isfile(FASTLIO_PCD):
            return run_pcd2pgm(name)
        return {'ok': False, 'error': 'fastlio 未运行且无 PCD 可转换'}

    # Remember mtime before kill so we wait for a fresh PCD
    prev_mtime = os.path.getmtime(FASTLIO_PCD) if os.path.isfile(FASTLIO_PCD) else 0
    stop_fastlio(signal.SIGINT)

    pcd_path = FASTLIO_PCD
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
        return {'ok': False, 'error': 'FAST-LIO 已停止但未生成 PCD'}
    return run_pcd2pgm(name, pcd_path)


def run_pcd2pgm(name, pcd_path=FASTLIO_PCD):
    os.makedirs(MAPS_DIR, exist_ok=True)
    out_base = os.path.join(MAPS_DIR, name)
    ex = load_lidar_extrinsic()
    # 大 PCD（建图走久会到 GB 级）必须体素降采样，否则 python 列表 OOM → 被杀 →「pgm not written」
    try:
        pcd_sz = os.path.getsize(pcd_path) if os.path.isfile(pcd_path) else 0
    except OSError:
        pcd_sz = 0
    voxel = '0.05' if pcd_sz > 200 * 1024 * 1024 else '0.04'
    timeout = 600 if pcd_sz > 400 * 1024 * 1024 else 300
    cmd = [
        'python3', PCD2PGM, pcd_path, out_base,
        '--pitch-down', str(ex.get('pitch_down', 0.7853981634)),
        '--roll', str(ex.get('roll', 0.0)),
        '--lidar-z', str(ex.get('lidar_z', 0.0) or 0.0),
        '--z-min', '0.15', '--z-max', '1.0',
        '--occ-min', '0.12', '--occ-pts', '4',
        '--free-pts', '1', '--ground-min-z', '-0.15',
        '--free-dilate', '8', '--free-erode', '2',
        '--despeckle', '2',
        '--voxel', voxel,
    ]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        log = (r.stdout + r.stderr).strip()
        ok = os.path.isfile(out_base + '.pgm')
        if ok:
            return {'ok': True, 'name': name + '.pgm', 'log': log[-2000:], 'error': None}
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


_loc_heal_ts = 0.0


def ensure_localization_stack(force=False):
    """Nav2 needs FAST-LIO (/Odometry,/cloud_registered) + lio_tf_bridge (odom→base_link)."""
    global _loc_heal_ts
    need = (not proc_alive('fastlio_mapping') or not proc_alive('lio_tf_bridge')
            or not proc_alive('livox_ros_driver2_node'))
    if need or force:
        # Cooldown avoids thrashing if LIO keeps dying (OOM / SIGKILL)
        now = time.time()
        if force or (now - _loc_heal_ts) > 8.0:
            _loc_heal_ts = now
            run_bg(f'bash {START_ALL} localization', '/tmp/localization.log')
            for _ in range(30):
                if proc_alive('fastlio_mapping') and proc_alive('lio_tf_bridge'):
                    break
                time.sleep(0.4)
    return {
        'fastlio': proc_alive('fastlio_mapping'),
        'lio_tf': proc_alive('lio_tf_bridge'),
        'livox': proc_alive('livox_ros_driver2_node'),
    }


def heal_nav_deps():
    """If Nav2 is up but LIO/scan/reloc died, bring them back (no dog motion)."""
    if not proc_alive('component_container_isolated'):
        return
    if not proc_alive('fastlio_mapping') or not proc_alive('lio_tf_bridge'):
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
    if proc_alive('auto_relocalize'):
        return True
    run_bg(
        'ros2 launch auto_relocalize auto_relocalize.launch.py',
        '/tmp/auto_relocalize.log')
    for _ in range(15):
        if proc_alive('auto_relocalize'):
            return True
        time.sleep(0.3)
    return proc_alive('auto_relocalize')


def call_relocalize(timeout=35.0):
    """Call /relocalize Trigger service (blocking, for HTTP API)."""
    loc = ensure_localization_stack()
    if not loc.get('fastlio'):
        return {'ok': False, 'error': 'FAST-LIO 未运行，无法重定位（先恢复激光里程计）', 'loc': loc}
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
        ok = 'success=True' in out or 'success=true' in out
        err = None
        if not ok:
            if 'ICP' in out or '请设初始位姿' in out or '初值' in out:
                err = 'ICP未对齐，请设初始位姿后自动精修'
            else:
                err = out[-300:] or 'relocalize failed'
        return {'ok': ok, 'log': out[-800:], 'error': err}
    except subprocess.TimeoutExpired:
        return {'ok': False, 'error': '重定位超时（地图/激光可能未就绪）'}
    except Exception as e:
        return {'ok': False, 'error': str(e)}


def reload_nav_map(body):
    """Hot-reload map via map_server and optionally force relocalize."""
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
    stop_nav2_procs()
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

    # Singleton Nav2 — previous double-launch left two navigate_to_pose servers
    stop_nav2_procs()
    # 导航 ≠ 建图：关掉建图预览；LIO 由 localization 模式占用（不做 map_building）
    kill_pattern('map_building_node')
    kill_pattern('map_building.launch')

    loc = ensure_localization_stack()
    if not loc['fastlio']:
        return {'ok': False, 'error': 'FAST-LIO 未能启动，导航需要里程计 TF（odom→base_link）',
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
    return {
        'ok': True, 'map': m, 'scan': scan_ok, 'loc': loc,
        'auto_relocalize': reloc_ok,
        'hint': '等待自动重定位（或手动设初始位姿）后再下目标',
    }


def restart_rosbridge():
    for pid in proc_pids('rosbridge_websocket'):
        try:
            os.kill(pid, signal.SIGKILL)
        except Exception:
            pass
    time.sleep(2)
    run_bg('ros2 launch rosbridge_server rosbridge_websocket_launch.xml', '/tmp/rosbridge.log')
    return {'ok': True}


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
        'fastlio': proc_alive('fastlio_mapping'),
        'lio_tf': proc_alive('lio_tf_bridge'),
        'map_building': proc_alive('map_building_node'),
        'scan': _scan_proc_alive(),
        'nav2': proc_alive('component_container_isolated'),
        'auto_relocalize': proc_alive('auto_relocalize'),
        'rosbridge': proc_alive('rosbridge_websocket'),
        'webops': proc_alive('web_ops_node'),
    }


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
        info['video_width'] = VIDEO_WIDTH
        info['video_fps'] = VIDEO_FPS
        info['video_q'] = VIDEO_Q
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

        self.pub_nav_status = self.create_publisher(String, '/web/nav_status', 10)
        self.pub_sys_status = self.create_publisher(String, '/web/sys_status', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # 网页 TFClient 常拿不到 map→base_link；这里查 TF 后转发给前端画激光
        self.pub_robot_pose = self.create_publisher(PoseStamped, '/web/robot_pose', 10)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_goal_handle = None
        self.nav_state = 'idle'

        self.create_timer(1.0, self.cb_status_timer)
        self.create_timer(0.1, self.cb_robot_pose_timer)
        self._set_nav_status('idle', '')

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
        msg.pose.position.z = t.transform.translation.z
        msg.pose.orientation = t.transform.rotation
        self.pub_robot_pose.publish(msg)

    def cb_nav_cmd(self, msg):
        x, y, yaw = msg.linear.x, msg.linear.y, msg.angular.z
        self.get_logger().info(f'nav_goal ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f}deg)')
        if abs(x) < 0.02 and abs(y) < 0.02:
            return
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self._set_nav_status('aborted', 'nav2_server_unavailable')
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        f = self.nav_client.send_goal_async(goal)
        f.add_done_callback(self._goal_sent_cb)

    def _goal_sent_cb(self, future):
        h = future.result()
        if not h.accepted:
            self._set_nav_status('aborted', 'goal_rejected')
            return
        self.nav_goal_handle = h
        self._set_nav_status('navigating', '')
        h.get_result_async().add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        st = future.result().status
        # GoalStatus: SUCCEEDED=4, CANCELED=5, ABORTED=6 (action_msgs)
        if st == 4:
            self._set_nav_status('reached', '')
        elif st == 5:
            self._set_nav_status('canceled', '')
        else:
            self._set_nav_status('aborted', f'status_{st}')
        self.nav_goal_handle = None

    def cb_nav_cancel(self, msg):
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
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

    def cb_init_pose(self, msg):
        x, y, yaw = msg.linear.x, msg.linear.y, msg.angular.z
        self.get_logger().info(f'init_pose ({x:.2f}, {y:.2f}, yaw {math.degrees(yaw):.0f}deg)')
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = float(x)
        pose.pose.pose.position.y = float(y)
        pose.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.25
        for _ in range(1):  # 只发一次；重定位节点会 ICP，连发会打三次
            self.pub_init.publish(pose)

    def cb_mapping(self, msg):
        if not proc_alive('fastlio_mapping'):
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

    def cb_stop_nav2(self, msg):
        stop_nav2_api()
        self._publish_zero()
        self._set_nav_status('idle', 'nav2_stopped')

    def cb_status_timer(self):
        try:
            heal_nav_deps()
        except Exception:
            pass
        status = service_status()
        status['nav'] = self.nav_state
        self.pub_sys_status.publish(String(data=json.dumps(status)))

    def _set_nav_status(self, state, detail):
        self.nav_state = state
        self.pub_nav_status.publish(
            String(data=state if not detail else f'{state}:{detail}'))


def main(args=None):
    rclpy.init(args=args)
    node = WebOpsNode()
    server = ThreadingHTTPServer(('0.0.0.0', 8090), ApiHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
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
