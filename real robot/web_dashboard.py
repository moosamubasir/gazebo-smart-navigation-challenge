#!/usr/bin/env python3
"""
web_dashboard.py — Combined: Web UI + Camera Grid Detector + ROS Publisher

Auto-detects white grid lines in the arena to compute exact perspective warp.
Falls back to full frame if auto-detection fails (no zoom).
"""
import threading
import time
import cv2
import numpy as np
import json
from collections import defaultdict

from flask import Flask, render_template_string, Response, jsonify, request
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

app = Flask(__name__)

# ── CAMERA ORIENTATION ────────────────────────────────────────────────────────
FLIP_MODE = 1   # 1 = horizontal flip. Try -1, 0, 1 if image is mirrored/rotated.

# ── COORDINATE MAPPING ────────────────────────────────────────────────────────
COORD_FLIP = 'v'

def _map(r, c):
    if COORD_FLIP == 'h':    return r,     4 - c
    if COORD_FLIP == 'v':    return 4 - r, c
    if COORD_FLIP == 'both': return 4 - r, 4 - c
    return r, c

def _reverse_map(ar, ac):
    if COORD_FLIP == 'h':    return ar,     4 - ac
    if COORD_FLIP == 'v':    return 4 - ar, ac
    if COORD_FLIP == 'both': return 4 - ar, 4 - ac
    return ar, ac

# ── HSV COLOUR THRESHOLDS ─────────────────────────────────────────────────────
RED_L1 = np.array([  0, 110,  60]); RED_U1 = np.array([ 10, 255, 255])
RED_L2 = np.array([165, 110,  60]); RED_U2 = np.array([180, 255, 255])
CYAN_L = np.array([ 88,  70,  90]); CYAN_U = np.array([102, 210, 255])
WOD_L  = np.array([ 10,  60, 120]); WOD_U  = np.array([ 22, 180, 220])
BOT_L  = np.array([  0,   50, 10]); BOT_U  = np.array([180,  80,  60])

BLOB_COLOR = 400
BLOB_WOOD  = 600
BLOB_ROBOT = 400

# ── VOTING ────────────────────────────────────────────────────────────────────
VOTE_WIN   = 4
_ov        = defaultdict(list)
_bv        = defaultdict(list)
_rv        = []
_collected = set()

# ── WARP STATE ────────────────────────────────────────────────────────────────
_warp_M      = None
_warp_cached = 0.0
WARP_SIZE    = (500, 500)
WARP_CACHE_S = 5.0   # re-detect every 5 seconds

# ── Dashboard state ───────────────────────────────────────────────────────────
dashboard_state = {
    "latest_frame":  None,
    "raw_frame":     None,
    "camera_status": "WAITING",
    "total_score":   50,
    "phase":         "SCANNING",
    "position_x":    0,
    "position_y":    0,
    "cam_position":  "—",
    "obstacles":     0,
    "bonuses_left":  0,
    "grid_matrix":   [[0]*5 for _ in range(5)],
    "logs": [{"time": "00:00:00", "tag": "System", "msg": "Dashboard initialised."}]
}
lock     = threading.Lock()
ros_node = None


# ── ROS Node ──────────────────────────────────────────────────────────────────

class DashboardNode(Node):
    def __init__(self):
        super().__init__('web_dashboard')
        self.obs_pub = self.create_publisher(String, '/grid/obstacles',  10)
        self.bon_pub = self.create_publisher(String, '/grid/bonuses',    10)
        self.bot_pub = self.create_publisher(String, '/grid/robot_cell', 10)
        self.cmd_pub = self.create_publisher(Twist,  '/cmd_vel',         10)
        self.create_subscription(String, '/robot/status', self._cb_status, 10)
        self.get_logger().info('DashboardNode ready.')

    def _cb_status(self, msg):
        global _collected
        if msg.data.startswith('Bonus at'):
            try:
                raw   = msg.data.replace('Bonus at (','').replace(')','').replace(' ','')
                parts = raw.split(',')
                _collected.add((int(parts[0]), int(parts[1])))
            except Exception:
                pass

    def publish_grid(self, obs_list, bon_list, bot_cell):
        try:
            o = String(); o.data = json.dumps(obs_list); self.obs_pub.publish(o)
            b = String(); b.data = json.dumps(bon_list); self.bon_pub.publish(b)
            if bot_cell is not None:
                r = String(); r.data = json.dumps(bot_cell); self.bot_pub.publish(r)
        except Exception as e:
            self.get_logger().warn(f'publish error: {e}', throttle_duration_sec=5.0)


# ── White-line grid auto-detection ───────────────────────────────────────────

def _detect_grid_corners(frame):
    """
    Detect the 4 outermost corners of the white grid using HoughLinesP.
    Finds all H+V line intersections and picks the 4 extreme corner points.
    Returns np.float32 array of [TL, TR, BR, BL] or None if detection fails.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape

    # Blur + threshold to isolate white lines
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 180, 255, cv2.THRESH_BINARY)

    # Morphology to connect broken white lines
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

    # Edge detect and find lines
    edges = cv2.Canny(thresh, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180,
                             threshold=60,
                             minLineLength=w // 8,
                             maxLineGap=30)
    if lines is None:
        return None

    # Separate into horizontal and vertical lines
    h_lines = []
    v_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle = abs(np.degrees(np.arctan2(y2 - y1, x2 - x1)))
        if angle < 25:
            h_lines.append((x1, y1, x2, y2))
        elif angle > 65:
            v_lines.append((x1, y1, x2, y2))

    if len(h_lines) < 2 or len(v_lines) < 2:
        return None

    # Find all intersections between horizontal and vertical lines
    def line_intersection(l1, l2):
        x1, y1, x2, y2 = l1
        x3, y3, x4, y4 = l2
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denom) < 1e-10:
            return None
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        ix = x1 + t * (x2 - x1)
        iy = y1 + t * (y2 - y1)
        if 0 <= ix <= w and 0 <= iy <= h:
            return (int(ix), int(iy))
        return None

    intersections = []
    for hl in h_lines:
        for vl in v_lines:
            pt = line_intersection(hl, vl)
            if pt:
                intersections.append(pt)

    if len(intersections) < 4:
        return None

    pts = np.array(intersections, dtype=np.float32)
    s   = pts.sum(axis=1)
    d   = pts[:, 0] - pts[:, 1]

    tl = pts[np.argmin(s)]   # smallest x+y = top-left
    br = pts[np.argmax(s)]   # largest  x+y = bottom-right
    tr = pts[np.argmax(d)]   # largest  x-y = top-right
    bl = pts[np.argmin(d)]   # smallest x-y = bottom-left

    # Sanity check: corners should span at least 20% of the frame
    if (br[0] - tl[0]) < w * 0.2 or (br[1] - tl[1]) < h * 0.2:
        return None

    return np.float32([tl, tr, br, bl])


# ── Perspective warp ──────────────────────────────────────────────────────────
def _detect_grid_corners(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape

    blur   = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 180, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
    edges  = cv2.Canny(thresh, 50, 150)
    lines  = cv2.HoughLinesP(edges, 1, np.pi/180,
                              threshold=60,
                              minLineLength=w//8,
                              maxLineGap=30)
    if lines is None:
        return None

    h_lines, v_lines = [], []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle = abs(np.degrees(np.arctan2(y2-y1, x2-x1)))
        if angle < 25:
            h_lines.append((x1, y1, x2, y2))
        elif angle > 65:
            v_lines.append((x1, y1, x2, y2))

    if len(h_lines) < 2 or len(v_lines) < 2:
        return None

    def intersect(l1, l2):
        x1,y1,x2,y2 = l1
        x3,y3,x4,y4 = l2
        denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
        if abs(denom) < 1e-10:
            return None
        t  = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / denom
        ix = x1 + t*(x2-x1)
        iy = y1 + t*(y2-y1)
        # Allow points slightly outside frame (offboard corners)
        if -w*0.3 <= ix <= w*1.3 and -h*0.3 <= iy <= h*1.3:
            return (int(ix), int(iy))
        return None

    intersections = []
    for hl in h_lines:
        for vl in v_lines:
            pt = intersect(hl, vl)
            if pt:
                intersections.append(pt)

    if len(intersections) < 4:
        return None

    pts = np.array(intersections, dtype=np.float32)
    s   = pts.sum(axis=1)
    d   = pts[:,0] - pts[:,1]

    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmax(d)]
    bl = pts[np.argmin(d)]

    # Sanity check
    if (br[0]-tl[0]) < w*0.15 or (br[1]-tl[1]) < h*0.15:
        return None

    return np.float32([tl, tr, br, bl])


def _get_warp_matrix(frame):
    global _warp_M, _warp_cached
    now = time.time()
    if _warp_M is not None and (now - _warp_cached) < WARP_CACHE_S:
        return _warp_M

    h, w = frame.shape[:2]
    W, H = WARP_SIZE
    dst  = np.float32([[0,0],[W,0],[W,H],[0,H]])

    corners = _detect_grid_corners(frame)
    if corners is not None:
        # Clamp offboard corners to frame boundary
        corners[:,0] = np.clip(corners[:,0], 0, w-1)
        corners[:,1] = np.clip(corners[:,1], 0, h-1)
        print(f'[WARP] Corners (clamped): {corners.tolist()}')
        _warp_M      = cv2.getPerspectiveTransform(corners, dst)
        _warp_cached = now
        return _warp_M

    # Fallback: full frame, no zoom
    print(f'[WARP] Fallback full frame {w}x{h}')
    src = np.float32([
        [w * 0.00, h * 0.00],   # Top-Left
        [w * 1.00, h * 0.00],   # Top-Right
        [w * 1.00, h * 1.00],   # Bottom-Right
        [w * 0.00, h * 1.00],   # Bottom-Left
    ])
    _warp_M      = cv2.getPerspectiveTransform(src, dst)
    _warp_cached = now
    return _warp_M


def _warp_frame(frame):
    frame = cv2.flip(frame, FLIP_MODE)
    M     = _get_warp_matrix(frame)
    return cv2.warpPerspective(frame, M, WARP_SIZE)


# ── Grid cell classifier ──────────────────────────────────────────────────────

def _classify_grid(grid):
    h, w  = grid.shape[:2]
    ch    = h // 5
    cw    = w // 5
    hsv   = cv2.cvtColor(grid, cv2.COLOR_BGR2HSV)
    dbg   = grid.copy()
    obs   = []
    bon   = []

    best_bot_px   = BLOB_ROBOT
    best_bot_cell = None

    # Draw aligned grid lines
    for i in range(6):
        cv2.line(dbg, (i * cw, 0), (i * cw, h), (60, 80, 100), 1)
        cv2.line(dbg, (0, i * ch), (w, i * ch), (60, 80, 100), 1)

    for img_r in range(5):
        for img_c in range(5):
            y1 = img_r * ch;       y2 = (img_r + 1) * ch
            x1 = img_c * cw;       x2 = (img_c + 1) * cw
            cx = (x1 + x2) // 2;  cy = (y1 + y2) // 2

            mg  = max(4, ch // 10)
            roi = hsv[y1 + mg:y2 - mg, x1 + mg:x2 - mg]
            if roi.size == 0:
                continue

            ar, ac = _map(img_r, img_c)
            label = None; color = None; bg = None

            # 1. Obstacle
            if cv2.countNonZero(cv2.inRange(roi, WOD_L, WOD_U)) > BLOB_WOOD:
                obs.append([ar, ac])
                label = 'OBS'; color = (0, 140, 255); bg = (40, 20, 0)

            # 2. RED bonus
            if label is None and (ar, ac) not in _collected:
                red = cv2.bitwise_or(cv2.inRange(roi, RED_L1, RED_U1),
                                     cv2.inRange(roi, RED_L2, RED_U2))
                if cv2.countNonZero(red) > BLOB_COLOR:
                    bon.append([ar, ac])
                    label = 'RED'; color = (50, 50, 220); bg = (0, 0, 40)

            # 3. CYAN bonus
            if label is None and (ar, ac) not in _collected:
                if cv2.countNonZero(cv2.inRange(roi, CYAN_L, CYAN_U)) > BLOB_COLOR:
                    bon.append([ar, ac])
                    label = 'CYAN'; color = (240, 240, 50); bg = (40, 40, 0)

            # 4. Robot
            if label is None:
                black_px = cv2.countNonZero(cv2.inRange(roi, BOT_L, BOT_U))
                if black_px > best_bot_px:
                    best_bot_px   = black_px
                    best_bot_cell = [ar, ac]

            # Debug overlay
            if label:
                if bg:
                    ov2 = dbg.copy()
                    cv2.rectangle(ov2, (x1+2, y1+2), (x2-2, y2-2), bg, -1)
                    cv2.addWeighted(ov2, 0.6, dbg, 0.4, 0, dbg)
                cv2.rectangle(dbg, (x1+2, y1+2), (x2-2, y2-2), color, 2)
                fs = 0.38; th = 1
                tw, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, fs, th)[0]
                cv2.putText(dbg, label, (cx - tw // 2, cy + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, fs, color, th)
            else:
                ph2, pw2 = roi.shape[:2]
                if ph2 > 0 and pw2 > 0:
                    hh, ss, vv = roi[ph2 // 2, pw2 // 2]
                    cv2.putText(dbg, f'{hh},{ss},{vv}', (x1+2, cy+4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.22, (50, 65, 80), 1)

            cv2.putText(dbg, f'{ar},{ac}', (x1+2, y1+9),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.26, (80, 120, 160), 1)

    # Draw robot marker
    bot = best_bot_cell
    if bot:
        img_r_d, img_c_d = _reverse_map(bot[0], bot[1])
        bx1 = img_c_d * cw;  bx2 = (img_c_d + 1) * cw
        by1 = img_r_d * ch;  by2 = (img_r_d + 1) * ch
        bcx = (bx1 + bx2) // 2;  bcy = (by1 + by2) // 2
        ov2 = dbg.copy()
        cv2.rectangle(ov2, (bx1+2, by1+2), (bx2-2, by2-2), (0, 30, 15), -1)
        cv2.addWeighted(ov2, 0.4, dbg, 0.6, 0, dbg)
        cv2.rectangle(dbg, (bx1+2, by1+2), (bx2-2, by2-2), (0, 255, 180), 3)
        pts = np.array([[bcx, by1+8], [bx2-8, bcy], [bcx, by2-8], [bx1+8, bcy]], np.int32)
        cv2.fillPoly(dbg, [pts], (0, 255, 180))
        cv2.putText(dbg, 'BOT', (bcx - 14, bcy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 20, 10), 1)

    return obs, bon, bot, dbg


# ── Main frame processing ─────────────────────────────────────────────────────

def process_frame(raw_bytes):
    global _ov, _bv, _rv, dashboard_state

    np_arr = np.frombuffer(raw_bytes, np.uint8)
    frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    if frame is None:
        return raw_bytes

    grid = _warp_frame(frame)
    obs, bon, bot, dbg = _classify_grid(grid)

    all_cells = [(r, c) for r in range(5) for c in range(5)]
    os_set    = {tuple(x) for x in obs}
    bs_set    = {tuple(x) for x in bon}

    for cell in all_cells:
        b = _ov[cell]; b.append(cell in os_set)
        if len(b) > VOTE_WIN: b.pop(0)
        b2 = _bv[cell]; b2.append(cell in bs_set)
        if len(b2) > VOTE_WIN: b2.pop(0)

    _rv.append(bot)
    if len(_rv) > VOTE_WIN: _rv.pop(0)

    if len(_rv) >= VOTE_WIN:
        thr   = VOTE_WIN // 2 + 1
        s_obs = [list(c) for c in all_cells if sum(_ov[c]) >= thr]
        s_bon = [list(c) for c in all_cells
                 if sum(_bv[c]) >= thr and tuple(c) not in _collected]

        vr    = [v for v in _rv if v is not None]
        s_bot = None
        if vr:
            s_bot = list(max(set(map(tuple, vr)), key=lambda x: vr.count(list(x))))

        if ros_node is not None:
            ros_node.publish_grid(s_obs, s_bon, s_bot)

        grid_matrix = [[0]*5 for _ in range(5)]
        for or_, oc in s_obs:
            if 0 <= or_ < 5 and 0 <= oc < 5:
                grid_matrix[or_][oc] = 1
        for br_, bc in s_bon:
            if 0 <= br_ < 5 and 0 <= bc < 5:
                grid_matrix[br_][bc] = 3
        rx, ry = 0, 0
        if s_bot and 0 <= s_bot[0] < 5 and 0 <= s_bot[1] < 5:
            rx = s_bot[1]; ry = s_bot[0]
            grid_matrix[s_bot[0]][s_bot[1]] = 2

        with lock:
            dashboard_state['grid_matrix']   = grid_matrix
            dashboard_state['position_x']    = rx
            dashboard_state['position_y']    = ry
            dashboard_state['cam_position']  = f'X:{rx} Y:{ry}'
            dashboard_state['obstacles']     = len(s_obs)
            dashboard_state['bonuses_left']  = len(s_bon)
            dashboard_state['camera_status'] = 'LIVE'

    _, jpeg = cv2.imencode('.jpg', dbg, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return jpeg.tobytes()


# ── Flask Routes ──────────────────────────────────────────────────────────────

@app.route('/')
def index():
    html = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>TCE Robot Arena Dashboard</title>
<style>
  :root{--bg:#070d14;--card:#0c1520;--border:#162536;--muted:#7f91a4;
        --cyan:#00e5ff;--mag:#e040fb;--red:#ff5252;--blue:#2979ff;
        --green:#00e676;--yellow:#ffd600;--orange:#ff9100;}
  *{box-sizing:border-box;margin:0;padding:0;font-family:'Segoe UI',sans-serif;}
  body{background:var(--bg);color:#fff;padding:12px;height:100vh;display:flex;
       flex-direction:column;overflow:hidden;}
  header{display:flex;justify-content:space-between;align-items:center;
         padding-bottom:12px;border-bottom:1px solid var(--border);}
  .logo{display:flex;align-items:center;gap:10px;}
  .logo-box{background:var(--cyan);color:var(--bg);font-weight:bold;
             padding:4px 8px;border-radius:3px;font-size:13px;}
  .logo-text{font-size:17px;font-weight:600;letter-spacing:1.5px;}
  .logo-text span{color:var(--cyan);}
  .phase-badge{border:1px solid var(--green);color:var(--green);
               padding:5px 18px;border-radius:4px;font-size:12px;font-weight:bold;}
  .workspace{display:flex;flex:1;gap:12px;margin-top:12px;height:calc(100vh - 72px);}
  .panel{background:var(--card);border:1px solid var(--border);
         border-radius:6px;padding:12px;display:flex;flex-direction:column;}
  .panel-title{font-size:10px;font-weight:600;color:var(--muted);
               letter-spacing:1.5px;text-transform:uppercase;margin-bottom:10px;
               border-left:2px solid var(--blue);padding-left:8px;}
  .left{width:220px;}
  .grid-wrap{flex:1;display:flex;align-items:center;justify-content:center;}
  table{width:100%;table-layout:fixed;border-collapse:separate;border-spacing:3px;}
  td{aspect-ratio:1;border:1px solid #1a2f47;background:rgba(16,28,44,.5);
     position:relative;border-radius:2px;vertical-align:middle;}
  td.robot{border:2px solid var(--cyan);box-shadow:inset 0 0 8px rgba(0,229,255,.25);}
  td.obs{background:rgba(255,82,82,.25);border-color:var(--red);}
  td.bonus{background:rgba(0,230,118,.18);border-color:var(--green);}
  .diamond{width:10px;height:10px;background:var(--cyan);margin:auto;
           transform:rotate(45deg);position:absolute;top:0;left:0;bottom:0;right:0;}
  .coord{position:absolute;bottom:2px;right:3px;font-size:7px;
         color:#385370;font-family:monospace;}
  .center{flex:1;}
  .video-box{flex:1;background:#000;border:1px solid var(--border);
             border-radius:4px;position:relative;display:flex;
             align-items:center;justify-content:center;overflow:hidden;}
  .feed-img{width:100%;height:100%;object-fit:contain;}
  .live-tag{position:absolute;top:12px;left:16px;color:var(--blue);
            font-size:10px;font-weight:bold;}
  .right{width:220px;gap:12px;}
  .score-box{text-align:center;padding:12px;border-bottom:1px solid var(--border);}
  .score-num{font-size:52px;font-family:monospace;color:var(--yellow);font-weight:bold;}
  .score-lbl{font-size:10px;color:var(--muted);letter-spacing:2px;}
  .metrics{flex:1;display:flex;flex-direction:column;gap:10px;margin-top:10px;font-size:12px;}
  .row{display:flex;justify-content:space-between;
       border-bottom:1px dashed #122030;padding-bottom:4px;}
  .lbl{color:var(--muted);font-size:11px;}
  .val{font-weight:bold;font-family:monospace;}
  .log{height:160px;background:#04080d;border:1px solid #111d2b;
       border-radius:4px;padding:8px;font-family:monospace;font-size:10px;
       overflow-y:auto;color:#4b627a;}
  .log span{color:#224061;margin-right:4px;}
  .hl{color:#738da8;}
</style>
</head>
<body>
<header>
  <div class="logo">
    <div class="logo-box">TCE</div>
    <div class="logo-text">Robot Arena <span>NAVIGATOR</span></div>
  </div>
  <button class="phase-badge" id="phase-btn">SCANNING</button>
</header>
<div class="workspace">
  <div class="panel left">
    <div class="panel-title">Arena Grid 5×5</div>
    <div class="grid-wrap"><table id="grid-table"></table></div>
  </div>
  <div class="panel center">
    <div class="panel-title">Camera Feed — Warped Grid View (A* Vision)</div>
    <div class="video-box">
      <div class="live-tag">&#9679; LIVE</div>
      <img class="feed-img" src="/video_feed" alt="Camera">
    </div>
  </div>
  <div class="panel right">
    <div class="score-box">
      <div class="score-num" id="score">0</div>
      <div class="score-lbl">TOTAL SCORE</div>
    </div>
    <div class="metrics">
      <div class="row"><div class="lbl">PHASE</div><div class="val" style="color:var(--cyan)" id="phase">—</div></div>
      <div class="row"><div class="lbl">POSITION (DR)</div><div class="val" style="color:var(--cyan)" id="pos">—</div></div>
      <div class="row"><div class="lbl">CAMERA POS</div><div class="val" id="campos">—</div></div>
      <div class="row"><div class="lbl">OBSTACLES</div><div class="val" style="color:var(--red)" id="obs">0</div></div>
      <div class="row"><div class="lbl">BONUSES LEFT</div><div class="val" style="color:var(--blue)" id="bon">0</div></div>
      <div class="row"><div class="lbl">CAMERA</div><div class="val" style="color:var(--mag)" id="cam">WAITING</div></div>
    </div>
    <div class="panel-title" style="margin-top:10px;">Event Log</div>
    <div class="log" id="log"></div>
  </div>
</div>
<script>
function poll(){
  fetch('/telemetry').then(r=>r.json()).then(d=>{
    document.getElementById('score').innerText  = d.total_score;
    document.getElementById('phase').innerText  = d.phase;
    document.getElementById('phase-btn').innerText = d.phase;
    document.getElementById('pos').innerText    = `(${d.position_x},${d.position_y})`;
    document.getElementById('campos').innerText = d.cam_position;
    document.getElementById('obs').innerText    = d.obstacles;
    document.getElementById('bon').innerText    = d.bonuses_left;
    document.getElementById('cam').innerText    = d.camera_status;
    const tbl = document.getElementById('grid-table');
    tbl.innerHTML='';
    for(let r=4;r>=0;r--){
      let row='<tr>';
      for(let c=0;c<=4;c++){
        let v=d.grid_matrix[r][c];
        let cls=v===2?'robot':v===1?'obs':v===3?'bonus':'';
        let inner=v===2?'<div class="diamond"></div>':'';
        row+=`<td class="${cls}">${inner}<div class="coord">${r},${c}</div></td>`;
      }
      tbl.innerHTML+=row+'</tr>';
    }
    const log=document.getElementById('log');
    log.innerHTML='';
    d.logs.forEach(l=>{
      let hl=l.tag?`<span class="hl">${l.tag}</span> `:'';
      log.innerHTML+=`<div><span>[${l.time}]</span>${hl}${l.msg}</div>`;
    });
  });
}
setInterval(poll,250);
</script>
</body>
</html>"""
    return render_template_string(html)


@app.route('/upload_frame', methods=['POST'])
def upload_frame():
    try:
        file_bytes = request.files['image'].read()
        # Save raw frame for /raw_frame and /debug_warp endpoints
        with lock:
            dashboard_state['raw_frame'] = file_bytes
        annotated = process_frame(file_bytes)
        with lock:
            dashboard_state['latest_frame']  = annotated
            dashboard_state['camera_status'] = 'LIVE'
        return jsonify({'status': 'captured'}), 200
    except Exception as e:
        return jsonify({'status': 'error', 'msg': str(e)}), 400


@app.route('/raw_frame')
def raw_frame():
    """Return latest raw (pre-warp) frame — used by calibrate_warp.py."""
    with lock:
        frame = dashboard_state.get('raw_frame')
    if frame is None:
        return jsonify({'error': 'no frame yet'}), 404
    return Response(frame, mimetype='image/jpeg')


@app.route('/debug_warp')
def debug_warp():
    """
    Shows the raw frame with detected grid corners drawn on it.
    Open http://<ip>:5000/debug_warp in browser to verify corner detection.
    Green quad = auto-detected corners. Red = fallback (full frame).
    """
    with lock:
        frame_bytes = dashboard_state.get('raw_frame')
    if not frame_bytes:
        return "No frame yet — start camera sender first.", 404
    np_arr = np.frombuffer(frame_bytes, np.uint8)
    frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    frame  = cv2.flip(frame, FLIP_MODE)
    h, w   = frame.shape[:2]

    corners = _detect_grid_corners(frame)
    if corners is not None:
        color = (0, 255, 0)
        label = 'AUTO-DETECTED'
    else:
        color = (0, 0, 255)
        label = 'FALLBACK (full frame)'
        corners = np.float32([[0,0],[w,0],[w,h],[0,h]])

    pts = corners.astype(np.int32)
    cv2.polylines(frame, [pts.reshape(-1,1,2)], True, color, 3)
    for i, (px, py) in enumerate(pts):
        cv2.circle(frame, (px, py), 10, color, -1)
        cv2.putText(frame, ['TL','TR','BR','BL'][i], (px+8, py+8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.putText(frame, label, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    _, jpeg = cv2.imencode('.jpg', frame)
    return Response(jpeg.tobytes(), mimetype='image/jpeg')


def _stream():
    while True:
        with lock:
            frame = dashboard_state['latest_frame']
        if frame is not None:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.033)


@app.route('/video_feed')
def video_feed():
    return Response(_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/telemetry')
def telemetry():
    with lock:
        state = dict(dashboard_state)
    state.pop('latest_frame', None)
    state.pop('raw_frame', None)
    return jsonify({k: (v.decode('utf-8', errors='ignore') if isinstance(v, bytes) else v)
                    for k, v in state.items()})


# ── ROS spin ──────────────────────────────────────────────────────────────────

def _ros_loop():
    global ros_node
    ros_node = DashboardNode()
    try:
        rclpy.spin(ros_node)
    except Exception as e:
        print(f'ROS error: {e}')
    finally:
        ros_node.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    threading.Thread(target=_ros_loop, daemon=True).start()
    print('Dashboard : http://0.0.0.0:5000')
    print('Debug warp: http://0.0.0.0:5000/debug_warp')
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()