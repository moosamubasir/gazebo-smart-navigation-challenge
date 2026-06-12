
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2, numpy as np, json, time
from collections import defaultdict

try:
    from cv_bridge import CvBridge
    HAS_BRIDGE = True
except ImportError:
    HAS_BRIDGE = False

GRID_ROWS = 5
GRID_COLS  = 5

# ── CAMERA ORIENTATION ───────────────────────────────────────────────────────
# -1 = 180° rotation (both flips). Adjust if grid appears mirrored.
FLIP_MODE = -1

# ── CALIBRATION SETTINGS ─────────────────────────────────────────────────────
# Set to True to print raw resolution + corner positions every frame.
# Set to False once you have filled in MANUAL_CORNERS below.
CALIBRATION_MODE = True

# Fill these in with the EXACT pixel (x, y) of each arena corner
# as seen in the RAW (unwarped, flipped) camera image.
# Order: Top-Left, Top-Right, Bottom-Right, Bottom-Left
MANUAL_CORNERS = None   # e.g. np.float32([[120,15],[610,15],[680,710],[50,710]])

# Fallback percentage-based corners used when MANUAL_CORNERS is None.
# These are intentionally wide — pull them in once you have real pixel values.
CORNER_TL = (0.05, 0.00)   # (x%, y%) Top-Left
CORNER_TR = (0.95, 0.00)   # (x%, y%) Top-Right
CORNER_BR = (0.98, 1.00)   # (x%, y%) Bottom-Right
CORNER_BL = (0.02, 1.00)   # (x%, y%) Bottom-Left

# ── COLOUR THRESHOLDS (HSV) ──────────────────────────────────────────────────
# Red bonus card (two hue bands due to HSV wrap-around)
RED_L1 = np.array([  0, 110,  60]); RED_U1 = np.array([ 10, 255, 255])
RED_L2 = np.array([165, 110,  60]); RED_U2 = np.array([180, 255, 255])

# Cyan/sky-blue bonus card
CYAN_L = np.array([ 88,  70,  90]); CYAN_U = np.array([102, 210, 255])

# Wood obstacles (light tan/brown)
WOD_L  = np.array([  8,  20, 140]); WOD_U  = np.array([ 25, 140, 255])

# Robot chassis: true black (low saturation AND low value)
BOT_L  = np.array([  0,   0,   0]); BOT_U  = np.array([180,  80,  60])

# Minimum pixel counts to confirm detection
BLOB_COLOR = 400   # bonus card minimum pixels
BLOB_WOOD  = 220   # obstacle minimum pixels
BLOB_ROBOT = 300   # robot chassis minimum pixels (largest black blob wins)

BROWN_L = np.array([15,  70, 100]); BROWN_U = np.array([28, 210, 200])
WHITE_L = np.array([ 0,   0, 165]); WHITE_U = np.array([180, 65, 255])
MIN_GRID = 3000; INSET = 15; VOTE_WIN = 4


class CameraVision(Node):

    def __init__(self):
        super().__init__('camera_vision')
        self.bridge = CvBridge() if HAS_BRIDGE else None

        self.obs_pub = self.create_publisher(String, '/grid/obstacles',  10)
        self.bon_pub = self.create_publisher(String, '/grid/bonuses',    10)
        self.bot_pub = self.create_publisher(String, '/grid/robot_cell', 10)
        self.dbg_pub = self.create_publisher(Image,  '/grid/debug_image', 10)

        self.create_subscription(Image,  '/image_raw',        self._cb, 1)
        self.create_subscription(Image,  '/camera/image_raw', self._cb, 1)
        self.create_subscription(String, '/robot/status',     self._cb_status, 10)
        self.create_subscription(String, '/robot/phase',      self._cb_phase,  10)

        self.collected: set = set()
        self._ov  = defaultdict(list)
        self._bv  = defaultdict(list)
        self._rv  = []
        self._fc  = 0
        self._ll  = time.time()
        self._last_proc_time = 0.0
        self._M   = None
        self._ws  = (500, 500)
        self._lw  = 0.0
        self._logged_res = False   # only log resolution once

        self.get_logger().info('camera_vision ready — robot BLACK detection + A* grid active.')
        if CALIBRATION_MODE:
            self.get_logger().warn(
                'CALIBRATION_MODE=True — watch the logs for RAW frame size and '
                'src_points, then set MANUAL_CORNERS and CALIBRATION_MODE=False.')

    # ── Status callbacks ──────────────────────────────────────────────────────

    def _cb_status(self, msg):
        if msg.data.startswith('Bonus at'):
            try:
                c = msg.data.replace('Bonus at (', '').replace(')', '').split(',')
                self.collected.add((int(c[0].strip()), int(c[1].strip())))
            except:
                pass

    def _cb_phase(self, msg):
        if msg.data == 'SCANNING':
            self.collected.clear()
            self._ov.clear(); self._bv.clear(); self._rv.clear()
            self._M = None; self._lw = 0.0

    # ── Main frame callback ───────────────────────────────────────────────────

    def _cb(self, msg: Image):
        now = time.time()
        if now - self._last_proc_time < 0.10:
            return
        frame = self._decode(msg)
        if frame is None:
            return
        self._last_proc_time = now

        # Correct camera orientation
        frame = cv2.flip(frame, FLIP_MODE)

        # ── Calibration logging ───────────────────────────────────────────────
        if CALIBRATION_MODE:
            h_img, w_img = frame.shape[:2]
            if not self._logged_res:
                self.get_logger().info(f'RAW frame size: {w_img} x {h_img} pixels')
                self._logged_res = True

        # Warp to clean top-down 500×500 grid view
        grid, method = self._extract(frame)

        # Classify all cells
        obs, bon, bot, dbg = self._classify(grid)

        # Always publish debug image
        self._pub_dbg(dbg, msg)

        # ── Voting to reduce flickering ───────────────────────────────────────
        all_cells = [(r, c) for r in range(5) for c in range(5)]
        os = {tuple(x) for x in obs}
        bs = {tuple(x) for x in bon}

        for cell in all_cells:
            b = self._ov[cell]; b.append(cell in os)
            if len(b) > VOTE_WIN: b.pop(0)
            b2 = self._bv[cell]; b2.append(cell in bs)
            if len(b2) > VOTE_WIN: b2.pop(0)

        self._rv.append(bot)
        if len(self._rv) > VOTE_WIN:
            self._rv.pop(0)

        if len(self._rv) < VOTE_WIN:
            return

        thr = VOTE_WIN // 2 + 1

        s_obs = [list(c) for c in all_cells if sum(self._ov[c]) >= thr]
        s_bon = [list(c) for c in all_cells
                 if sum(self._bv[c]) >= thr and tuple(c) not in self.collected]

        vr = [v for v in self._rv if v is not None]
        s_bot = None
        if vr:
            s_bot = max(set(map(tuple, vr)), key=lambda x: vr.count(list(x)))
            s_bot = list(s_bot)

        # Publish
        o_msg = String(); o_msg.data = json.dumps(s_obs); self.obs_pub.publish(o_msg)
        b_msg = String(); b_msg.data = json.dumps(s_bon); self.bon_pub.publish(b_msg)
        if s_bot:
            r_msg = String(); r_msg.data = json.dumps(s_bot); self.bot_pub.publish(r_msg)

        if time.time() - self._ll > 1.5:
            self.get_logger().info(
                f'[{method}] obs={s_obs} bon={s_bon} robot={s_bot}')
            self._ll = time.time()

    # ── Grid extraction (perspective warp) ───────────────────────────────────

    def _extract(self, frame):
        h_img, w_img = frame.shape[:2]
        W, H = self._ws
        dst_points = np.float32([[0, 0], [W, 0], [W, H], [0, H]])

        # ── Option A: exact pixel corners (most accurate) ─────────────────────
        if MANUAL_CORNERS is not None:
            src_points = MANUAL_CORNERS.astype(np.float32)
            if CALIBRATION_MODE:
                self.get_logger().info(
                    f'Using MANUAL_CORNERS: {src_points.tolist()}', throttle_duration_sec=3.0)
            self._M = cv2.getPerspectiveTransform(src_points, dst_points)
            return cv2.warpPerspective(frame, self._M, self._ws), 'MANUAL_PX'

        # ── Option B: percentage-based corners (fallback) ─────────────────────
        src_points = np.float32([
            [int(w_img * CORNER_TL[0]), int(h_img * CORNER_TL[1])],  # Top-Left
            [int(w_img * CORNER_TR[0]), int(h_img * CORNER_TR[1])],  # Top-Right
            [int(w_img * CORNER_BR[0]), int(h_img * CORNER_BR[1])],  # Bottom-Right
            [int(w_img * CORNER_BL[0]), int(h_img * CORNER_BL[1])],  # Bottom-Left
        ])
        if CALIBRATION_MODE:
            self.get_logger().info(
                f'RAW={w_img}x{h_img}  src_points={src_points.tolist()}',
                throttle_duration_sec=3.0)
        self._M = cv2.getPerspectiveTransform(src_points, dst_points)
        return cv2.warpPerspective(frame, self._M, self._ws), 'PCT'

    # ── (Unused dynamic detection kept for reference) ─────────────────────────

    def _brown(self, frame, hsv):
        mask = cv2.inRange(hsv, BROWN_L, BROWN_U)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, k)
        cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cs:
            return None
        c = max(cs, key=cv2.contourArea)
        if cv2.contourArea(c) < MIN_GRID:
            return None
        p = cv2.arcLength(c, True); ap = cv2.approxPolyDP(c, 0.02 * p, True)
        if len(ap) == 4:
            corners = ap.reshape(4, 2).astype(np.float32)
        else:
            x, y, w, h = cv2.boundingRect(c); ins = INSET
            corners = np.float32([[x + ins, y + ins], [x + w - ins, y + ins],
                                  [x + w - ins, y + h - ins], [x + ins, y + h - ins]])
        corners = self._order(corners)
        W, H = self._ws
        return cv2.getPerspectiveTransform(
            corners, np.float32([[0, 0], [W, 0], [W, H], [0, H]]))

    def _white(self, frame, hsv):
        mask = cv2.inRange(hsv, WHITE_L, WHITE_U)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cs:
            return None
        xn, yn = frame.shape[1], frame.shape[0]; xx, yx = 0, 0; tot = 0
        for c in cs:
            x, y, w, h = cv2.boundingRect(c); tot += w * h
            xn = min(xn, x); yn = min(yn, y); xx = max(xx, x + w); yx = max(yx, y + h)
        if tot < MIN_GRID:
            return None
        return (xn, yn, xx - xn, yx - yn)

    # ── Cell classification ───────────────────────────────────────────────────

    def _classify(self, grid):
        h, w = grid.shape[:2]; ch = h // 5; cw = w // 5
        hsv = cv2.cvtColor(grid, cv2.COLOR_BGR2HSV)
        dbg = grid.copy()
        obs = []; bon = []; bot = None

        best_bot_px = BLOB_ROBOT
        best_bot_cell = None

        for r in range(5):
            for c in range(5):
                y1 = r * ch; y2 = (r + 1) * ch
                x1 = c * cw; x2 = (c + 1) * cw
                cx = (x1 + x2) // 2; cy = (y1 + y2) // 2

                cv2.rectangle(dbg, (x1, y1), (x2, y2), (30, 40, 55), 1)

                mg = max(6, ch // 8)
                roi = hsv[y1 + mg:y2 - mg, x1 + mg:x2 - mg]
                if roi.size == 0:
                    continue

                label = None; color = None; bg = None

                # ── 1. Obstacle (wood/tan) ────────────────────────────────────
                cnt = cv2.countNonZero(cv2.inRange(roi, WOD_L, WOD_U))
                if cnt > BLOB_WOOD:
                    obs.append([r, c])
                    label = 'OBS'; color = (0, 140, 255); bg = (40, 20, 0)

                # ── 2. RED bonus card ─────────────────────────────────────────
                if label is None and (r, c) not in self.collected:
                    red = cv2.bitwise_or(
                        cv2.inRange(roi, RED_L1, RED_U1),
                        cv2.inRange(roi, RED_L2, RED_U2))
                    if cv2.countNonZero(red) > BLOB_COLOR:
                        bon.append([r, c])
                        label = 'RED'; color = (50, 50, 220); bg = (0, 0, 40)

                # ── 3. CYAN bonus card ────────────────────────────────────────
                if label is None and (r, c) not in self.collected:
                    if cv2.countNonZero(cv2.inRange(roi, CYAN_L, CYAN_U)) > BLOB_COLOR:
                        bon.append([r, c])
                        label = 'CYAN'; color = (240, 240, 50); bg = (40, 40, 0)

                # ── 4. Robot (black chassis) ──────────────────────────────────
                if label is None:
                    black_px = cv2.countNonZero(cv2.inRange(roi, BOT_L, BOT_U))
                    if black_px > best_bot_px:
                        best_bot_px = black_px
                        best_bot_cell = [r, c]

                # ── Debug rendering ───────────────────────────────────────────
                if label:
                    if bg:
                        overlay = dbg.copy()
                        cv2.rectangle(overlay, (x1 + 2, y1 + 2), (x2 - 2, y2 - 2), bg, -1)
                        cv2.addWeighted(overlay, 0.6, dbg, 0.4, 0, dbg)
                    cv2.rectangle(dbg, (x1 + 2, y1 + 2), (x2 - 2, y2 - 2), color, 2)
                    fs = 0.38; th = 1
                    tw, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, fs, th)[0]
                    cv2.putText(dbg, label, (cx - tw // 2, cy + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, fs, color, th)
                else:
                    ph2, pw2 = roi.shape[:2]
                    if ph2 > 0 and pw2 > 0:
                        hh, ss, vv = roi[ph2 // 2, pw2 // 2]
                        cv2.putText(dbg, f'{hh},{ss},{vv}', (x1 + 2, cy + 4),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.24, (50, 65, 80), 1)

                cv2.putText(dbg, f'{r},{c}', (x1 + 2, y1 + 9),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.24, (40, 55, 70), 1)

        # ── Draw robot in the winning cell ────────────────────────────────────
        bot = best_bot_cell
        if bot:
            br, bc = bot
            bx1 = bc * cw; bx2 = (bc + 1) * cw
            by1 = br * ch; by2 = (br + 1) * ch
            bcx = (bx1 + bx2) // 2; bcy = (by1 + by2) // 2

            overlay = dbg.copy()
            cv2.rectangle(overlay, (bx1 + 2, by1 + 2), (bx2 - 2, by2 - 2), (20, 30, 0), -1)
            cv2.addWeighted(overlay, 0.4, dbg, 0.6, 0, dbg)
            cv2.rectangle(dbg, (bx1 + 2, by1 + 2), (bx2 - 2, by2 - 2), (0, 255, 200), 3)

            pts = np.array([[bcx, by1 + 8], [bx2 - 8, bcy],
                            [bcx, by2 - 8], [bx1 + 8, bcy]], np.int32)
            cv2.fillPoly(dbg, [pts], (0, 255, 200))
            cv2.putText(dbg, 'BOT', (bcx - 12, bcy + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 20, 10), 1)

        return obs, bon, bot, dbg

    # ── Publish debug image ───────────────────────────────────────────────────

    def _pub_dbg(self, dbg, orig):
        try:
            msg = Image()
            msg.header = orig.header
            msg.height = dbg.shape[0]; msg.width = dbg.shape[1]
            msg.encoding = 'bgr8'; msg.step = dbg.shape[1] * 3
            msg.data = dbg.tobytes()
            self.dbg_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'dbg pub: {e}', throttle_duration_sec=5.0)

    @staticmethod
    def _order(pts):
        pts = np.array(pts, dtype=np.float32).reshape(4, 2)
        r = np.zeros((4, 2), dtype=np.float32)
        s = pts.sum(axis=1)
        r[0] = pts[np.argmin(s)]
        r[2] = pts[np.argmax(s)]
        d = pts[:, 0] - pts[:, 1]
        r[1] = pts[np.argmin(d)]
        r[3] = pts[np.argmax(d)]
        return r

    def _decode(self, msg: Image):
        enc = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            if enc == 'rgb8':
                return cv2.cvtColor(data.reshape((msg.height, msg.width, 3)), cv2.COLOR_RGB2BGR)
            if enc == 'bgr8':
                return data.reshape((msg.height, msg.width, 3))
            if any(x in enc for x in ('yuyv', 'yuv422', 'yuy2')):
                return cv2.cvtColor(data.reshape((msg.height, msg.width, 2)), cv2.COLOR_YUV2BGR_YUYV)
            if enc in ('mono8', '8uc1'):
                return cv2.cvtColor(data.reshape((msg.height, msg.width)), cv2.COLOR_GRAY2BGR)
            if HAS_BRIDGE and self.bridge:
                return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if img is not None:
                return img
        except Exception as e:
            self.get_logger().warn(f'decode ({enc}): {e}', throttle_duration_sec=5.0)
        return None


def main(args=None):
    rclpy.init(args=args)
    node = CameraVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()