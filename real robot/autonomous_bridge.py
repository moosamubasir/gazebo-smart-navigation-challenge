

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import json, time, heapq, threading

# ── CONSTANTS ─────────────────────────────────────────────────────────────────
GRID         = 5
GOAL         = (4, 4)
START        = (0, 0)
SCAN_DURATION = 8.0    # seconds to scan before planning

# ── TIMING (tune to your hardware) ────────────────────────────────────────────
STOP_GAP     = 0.2    # pause between commands
MOVE_TIMEOUT = 10.0   # max wait for Arduino DONE


# ── A* ────────────────────────────────────────────────────────────────────────
def astar(start, goal, obstacles):
    blocked = obstacles - {goal}

    def h(n):
        return abs(n[0]-goal[0]) + abs(n[1]-goal[1])

    heap   = [(h(start), 0, start, [start])]
    best_g = {}
    while heap:
        f, g, cur, path = heapq.heappop(heap)
        if best_g.get(cur, float('inf')) <= g:
            continue
        best_g[cur] = g
        if cur == goal:
            return path
        x, y = cur
        for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)]:
            nx, ny = x+dx, y+dy
            nxt = (nx, ny)
            if not (0 <= nx < GRID and 0 <= ny < GRID):
                continue
            if nxt in blocked:
                continue
            ng = g + 1
            if best_g.get(nxt, float('inf')) <= ng:
                continue
            heapq.heappush(heap, (ng+h(nxt), ng, nxt, path+[nxt]))
    return None


def plan_full_route(start, bonuses, obstacles):
    """
    Greedy nearest-first TSP over bonuses, then GOAL.
    Returns a flat list of (x,y) waypoints from start to GOAL.
    """
    remaining = list(bonuses)
    pos       = start
    full_path = []

    while remaining:
        # pick nearest reachable bonus
        best, best_path = None, None
        for b in remaining:
            p = astar(pos, b, obstacles)
            if p:
                if best_path is None or len(p) < len(best_path):
                    best, best_path = b, p
        if best is None:
            break   # no reachable bonus left — go straight to goal
        # append path (skip first cell = current pos, already in list)
        full_path += best_path[1:]
        remaining.remove(best)
        pos = best

    # final leg to GOAL
    p = astar(pos, GOAL, obstacles)
    if p:
        full_path += p[1:]

    return full_path


# ── NODE ──────────────────────────────────────────────────────────────────────
class AutonomousRobot(Node):

    HEADINGS  = ['N', 'E', 'S', 'W']

    def __init__(self):
        super().__init__('autonomous_robot')

        self.cmd_pub  = self.create_publisher(Twist,  '/cmd_vel',      10)
        self.stat_pub = self.create_publisher(String, '/robot/status', 10)

        # ── State ────────────────────────────────────────────────────────────
        self.robot_pos     = START
        self.heading       = 'N'
        self.obstacles     : set  = set()
        self.bonuses       : set  = set()   # initial bonuses from scan
        self.collected     : set  = set()   # never revisit these
        self.mission_done  = False

        # ── Scan gate ────────────────────────────────────────────────────────
        self._scan_start  = time.time()
        self._got_obs     = False
        self._got_bon     = False
        self._got_bot     = False
        self._planned     = False           # True once full route is locked in

        # ── Locked route (set once after scan) ───────────────────────────────
        self._route       = []              # flat list of (x,y) cells to visit
        self._route_idx   = 0              # next cell index in _route

        # ── Hardware sync ────────────────────────────────────────────────────
        self._move_done   = threading.Event()

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(String, '/grid/obstacles',  self._cb_obs,  10)
        self.create_subscription(String, '/grid/bonuses',    self._cb_bon,  10)
        self.create_subscription(String, '/grid/robot_cell', self._cb_bot,  10)
        self.create_subscription(String, '/robot/move_done', self._cb_done, 10)

        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f'AutonomousRobot ready — scanning for {SCAN_DURATION}s...')

    # ── COORDINATE DECODE ─────────────────────────────────────────────────────
    @staticmethod
    def _decode(data):
        cells = json.loads(data)
        return {(ac, ar) for ar, ac in cells}

    # ── CALLBACKS (scan phase only) ───────────────────────────────────────────
    def _cb_obs(self, msg):
        self.obstacles = self._decode(msg.data)
        if not self._got_obs:
            self._got_obs = True
            self.get_logger().info(f'[SCAN] Obstacles: {self.obstacles}')

    def _cb_bon(self, msg):
        # Keep adding newly seen bonuses UNTIL planning is locked
        if not self._planned:
            fresh = self._decode(msg.data) - self.collected
            self.bonuses |= fresh          # accumulate across frames
            if not self._got_bon and fresh:
                self._got_bon = True
                self.get_logger().info(f'[SCAN] Bonuses so far: {self.bonuses}')

    def _cb_bot(self, msg):
        # Only use camera for initial position during scan
        if self._planned:
            return
        try:
            rc   = json.loads(msg.data)
            ar, ac = rc[0], rc[1]
            phy  = (ac, ar)
            # Filter bad readings: (4,4) or (0,0) when we know we're at start
            if phy == GOAL:
                return
            if not self._got_bot:
                self._got_bot = True
                self.get_logger().info(
                    f'[SCAN] Camera active, first reading: {phy} '
                    f'(robot locked at {START})')
        except Exception as e:
            self.get_logger().warn(f'bot parse: {e}')

    def _cb_done(self, msg):
        self._move_done.set()

    # ── MAIN TICK ─────────────────────────────────────────────────────────────
    def _tick(self):
        if self.mission_done:
            return

        elapsed = time.time() - self._scan_start

        # ── Scan phase ───────────────────────────────────────────────────────
        if not self._planned:
            remaining = max(0.0, SCAN_DURATION - elapsed)
            if elapsed < SCAN_DURATION:
                self.get_logger().info(
                    f'[SCAN] {remaining:.1f}s left | '
                    f'obs={self._got_obs} bon={self._got_bon} cam={self._got_bot}',
                    throttle_duration_sec=2.0)
                return

            # ── Scan done — plan the full route now ──────────────────────────
            if not self.bonuses:
                self.get_logger().warn('[PLAN] No bonuses detected — going straight to GOAL')
            self._route = plan_full_route(START, self.bonuses, self.obstacles)
            self._route_idx = 0
            self._planned   = True

            self.get_logger().info('=' * 55)
            self.get_logger().info('✔  ROUTE LOCKED — executing now (no re-planning)')
            self.get_logger().info(f'   Start     : {START}')
            self.get_logger().info(f'   Obstacles : {self.obstacles}')
            self.get_logger().info(f'   Bonuses   : {self.bonuses}')
            self.get_logger().info(f'   Route     : {self._route}')
            self.get_logger().info('=' * 55)
            self._publish_status('GRID_READY')

            # Kick off execution in a background thread
            threading.Thread(target=self._execute_route, daemon=True).start()

    # ── ROUTE EXECUTION ───────────────────────────────────────────────────────
    def _execute_route(self):
        """Walk every cell in _route in order. No camera, no re-planning."""
        pos = START

        for nxt in self._route:
            if self.mission_done:
                return

            self.get_logger().info(f'  STEP {pos} → {nxt}')
            self._move_to(pos, nxt)
            pos = nxt
            self.robot_pos = pos

            # ── Collect bonus if we just landed on one ───────────────────────
            if pos in self.bonuses and pos not in self.collected:
                self.collected.add(pos)
                self.bonuses.discard(pos)
                self._publish_status(f'Bonus at ({pos[0]},{pos[1]})')
                self.get_logger().info(
                    f'  ★ Collected bonus at {pos}! '
                    f'Still to collect: {self.bonuses - self.collected}')

        # ── Route finished ───────────────────────────────────────────────────
        if pos == GOAL:
            self.get_logger().info('★ MISSION COMPLETE ★')
            self.mission_done = True
            self._stop()
            self._publish_status('MISSION_COMPLETE')
        else:
            self.get_logger().warn(
                f'Route ended at {pos}, not at GOAL — check planning.')

    # ── SINGLE STEP ───────────────────────────────────────────────────────────
    def _move_to(self, cur, nxt):
        dx, dy = nxt[0]-cur[0], nxt[1]-cur[1]
        needed = {(0,1):'N', (0,-1):'S', (1,0):'E', (-1,0):'W'}[(dx, dy)]

        for d in self._turns_to(self.heading, needed):
            self.get_logger().info(f'    Turn {d}')
            self._turn(d)
            time.sleep(STOP_GAP)
        self.heading = needed

        self.get_logger().info(f'    Forward')
        self._forward()
        time.sleep(STOP_GAP)

    # ── TURN SEQUENCE ─────────────────────────────────────────────────────────
    def _turns_to(self, cur, tgt):
        ci   = self.HEADINGS.index(cur)
        ti   = self.HEADINGS.index(tgt)
        diff = (ti - ci) % 4
        if diff == 0: return []
        if diff == 1: return ['R']
        if diff == 2: return ['R', 'R']
        if diff == 3: return ['L']
        return []

    # ── HARDWARE ──────────────────────────────────────────────────────────────
    def _wait_done(self):
        self._move_done.clear()
        if not self._move_done.wait(timeout=MOVE_TIMEOUT):
            self.get_logger().warn('Arduino DONE timeout')

    def _turn(self, d):
        t = Twist()
        t.angular.z = -1.0 if d == 'R' else 1.0
        self.cmd_pub.publish(t)
        self._wait_done()
        self._stop()

    def _forward(self):
        t = Twist()
        t.linear.x = 0.15
        self.cmd_pub.publish(t)
        self._wait_done()
        self._stop()

    def _stop(self):
        self.cmd_pub.publish(Twist())
        time.sleep(STOP_GAP)

    def _publish_status(self, msg):
        s = String(); s.data = msg
        self.stat_pub.publish(s)
        self.get_logger().info(f'[STATUS] {msg}')


# ── ENTRY ─────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = AutonomousRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()