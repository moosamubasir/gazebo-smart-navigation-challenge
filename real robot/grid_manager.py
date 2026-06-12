#!/usr/bin/env python3
"""
grid_manager.py  —  follow_grid package 

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import String

import json
import math
import heapq
from typing import Optional


# ─── Constants ───────────────────────────────────────────────────────────────

GRID_SIZE   = 5
CELL_SIZE_M = 0.10

CELL_FREE     = 0
CELL_OBSTACLE = 1
CELL_BONUS_G  = 2
CELL_BONUS_R  = 3
CELL_ROBOT    = 4
CELL_GOAL     = 5

START = (0, 0)
GOAL  = (4, 4)

MOVE_COST        = 10
GREEN_BONUS_COST = 2
RED_PENALTY_COST = 25
LIDAR_OBS_RANGE  = 0.25


# ─── Grid ────────────────────────────────────────────────────────────────────

class Grid:
    def __init__(self):
        self.cells = [[CELL_FREE] * GRID_SIZE for _ in range(GRID_SIZE)]
        self.cells[GOAL[1]][GOAL[0]] = CELL_GOAL
        self.score = 0

    def get(self, col, row):
        if 0 <= col < GRID_SIZE and 0 <= row < GRID_SIZE:
            return self.cells[row][col]
        return CELL_OBSTACLE

    def set(self, col, row, val):
        if 0 <= col < GRID_SIZE and 0 <= row < GRID_SIZE:
            self.cells[row][col] = val

    def mark_obstacle(self, col, row):
        if (col, row) not in (GOAL, START):
            if self.get(col, row) in (CELL_FREE, CELL_ROBOT):
                self.set(col, row, CELL_OBSTACLE)

    def mark_bonus(self, col, row, color):
        if self.get(col, row) in (CELL_FREE, CELL_ROBOT):
            self.set(col, row, CELL_BONUS_G if color == 'green' else CELL_BONUS_R)

    def collect(self, col, row):
        c = self.get(col, row)
        if c == CELL_BONUS_G:
            self.set(col, row, CELL_FREE)
            self.score += 10
            return 'green'
        if c == CELL_BONUS_R:
            self.set(col, row, CELL_FREE)
            self.score -= 5
            return 'red'
        return None

    def passable(self, col, row):
        return self.get(col, row) != CELL_OBSTACLE

    def entry_cost(self, col, row):
        c = self.get(col, row)
        if c == CELL_OBSTACLE: return float('inf')
        if c == CELL_BONUS_G:  return GREEN_BONUS_COST
        if c == CELL_BONUS_R:  return RED_PENALTY_COST
        return MOVE_COST

    def to_json(self):
        return json.dumps(self.cells)


# ─── A* ──────────────────────────────────────────────────────────────────────

def astar(grid: Grid, start, goal):
    h = lambda a, b: (abs(a[0]-b[0]) + abs(a[1]-b[1])) * MOVE_COST
    heap = [(h(start, goal), 0.0, start[0], start[1])]
    came = {start: None}
    g    = {start: 0.0}
    dirs = [(1,0),(-1,0),(0,1),(0,-1)]

    while heap:
        f, gc, col, row = heapq.heappop(heap)
        cur = (col, row)
        if cur == goal:
            path = []
            while cur:
                path.append(cur)
                cur = came[cur]
            path.reverse()
            return path
        if gc > g.get(cur, float('inf')):
            continue
        for dc, dr in dirs:
            nb = (col+dc, row+dr)
            if not grid.passable(nb[0], nb[1]):
                continue
            tg = gc + grid.entry_cost(nb[0], nb[1])
            if tg < g.get(nb, float('inf')):
                g[nb] = tg
                came[nb] = cur
                heapq.heappush(heap, (tg + h(nb, goal), tg, nb[0], nb[1]))
    return None


# ─── Node ────────────────────────────────────────────────────────────────────

class GridManagerNode(Node):

    def __init__(self):
        super().__init__('grid_manager')
        self.get_logger().info('Grid Manager Node starting...')

        self.grid      = Grid()
        self.robot_pos = START
        self.path      = []
        self.status    = 'PLANNING'

        # ── Publishers ──
        self.pub_grid   = self.create_publisher(String, '/grid/occupancy',     10)
        self.pub_path   = self.create_publisher(String, '/grid/path',          10)
        self.pub_wp     = self.create_publisher(Point,  '/grid/next_waypoint', 10)
        self.pub_status = self.create_publisher(String, '/grid/status',        10)

        # ── Subscribers ──
        self.create_subscription(LaserScan, '/scan',
                                 self._cb_lidar,  10)
        self.create_subscription(String, '/vision/bonus_cells',
                                 self._cb_bonus,  10)
        self.create_subscription(Point,  '/vision/robot_grid_pos',
                                 self._cb_pos,    10)
        # Also listen to odom as fallback position update
        from nav_msgs.msg import Odometry
        self.create_subscription(Odometry, '/odom',
                                 self._cb_odom,   10)

        # Plan + publish waypoint every 500ms
        self.create_timer(0.5, self._planning_loop)

        self.get_logger().info('Grid Manager ready. START=(0,0) GOAL=(4,4)')

    # ─────────────────────────────────────────────────────────────────────────
    def _cb_lidar(self, msg: LaserScan):
        angle = msg.angle_min
        rc, rr = self.robot_pos
        for dist in msg.ranges:
            angle += msg.angle_increment
            if math.isnan(dist) or math.isinf(dist):
                continue
            if dist < msg.range_min or dist > 2.0:
                continue
            if dist < LIDAR_OBS_RANGE:
                oc = rc + round(math.cos(angle) * dist / CELL_SIZE_M)
                or_ = rr + round(math.sin(angle) * dist / CELL_SIZE_M)
                self.grid.mark_obstacle(int(oc), int(or_))

    def _cb_bonus(self, msg: String):
        try:
            for cell in json.loads(msg.data):
                self.grid.mark_bonus(int(cell['col']), int(cell['row']), cell['color'])
        except Exception:
            pass

    def _cb_pos(self, msg: Point):
        col = max(0, min(GRID_SIZE-1, int(round(msg.x))))
        row = max(0, min(GRID_SIZE-1, int(round(msg.y))))
        if (col, row) != self.robot_pos:
            self.get_logger().info(f'Robot moved to ({col},{row})')
            self.robot_pos = (col, row)
            self._on_enter(col, row)

    def _cb_odom(self, msg):
        """Update robot grid position from wheel odometry."""
        x_m = msg.pose.pose.position.x
        y_m = msg.pose.pose.position.y
        col = max(0, min(GRID_SIZE-1, int(round(x_m / CELL_SIZE_M))))
        row = max(0, min(GRID_SIZE-1, int(round(y_m / CELL_SIZE_M))))
        if (col, row) != self.robot_pos:
            self.robot_pos = (col, row)
            self._on_enter(col, row)

    def _on_enter(self, col, row):
        collected = self.grid.collect(col, row)
        if collected == 'green':
            self.get_logger().info(f'GREEN bonus at ({col},{row}) score={self.grid.score}')
            self._pub_status('BONUS_GREEN')
        elif collected == 'red':
            self.get_logger().info(f'RED penalty at ({col},{row}) score={self.grid.score}')
            self._pub_status('BONUS_RED')
        if (col, row) == GOAL:
            self.get_logger().info(f'GOAL REACHED! Score={self.grid.score}')
            self._pub_status('ARRIVED')
            self.status = 'ARRIVED'

    # ─────────────────────────────────────────────────────────────────────────
    def _planning_loop(self):
        if self.status == 'ARRIVED':
            return

        # Update robot cell on grid display
        for r in range(GRID_SIZE):
            for c in range(GRID_SIZE):
                if self.grid.get(c, r) == CELL_ROBOT:
                    self.grid.set(c, r, CELL_FREE)
        self.grid.set(self.robot_pos[0], self.robot_pos[1], CELL_ROBOT)

        path = astar(self.grid, self.robot_pos, GOAL)

        if path is None:
            self.get_logger().warn('No path found!')
            self._pub_status('BLOCKED')
            self._pub_grid()
            return

        self.path   = path
        self.status = 'MOVING'

        self.get_logger().info(
            f'Path ({len(path)} steps): {path} | '
            f'Green bonuses on path: {[p for p in path if self.grid.get(p[0],p[1])==CELL_BONUS_G]} | '
            f'Score so far: {self.grid.score}')

        # Publish full path
        pm      = String()
        pm.data = json.dumps([[c, r] for c, r in path])
        self.pub_path.publish(pm)

        # ── Publish next waypoint EVERY cycle ──
        # hardware_bridge only moves if not already moving,
        # so publishing repeatedly is safe and ensures it doesn't miss it
        if len(path) > 1:
            nc, nr = path[1]
            wp = Point()
            wp.x = float(nc)
            wp.y = float(nr)
            wp.z = 0.0
            self.pub_wp.publish(wp)

        self._pub_grid()
        self._pub_status(self.status)

    def _pub_grid(self):
        m = String(); m.data = self.grid.to_json()
        self.pub_grid.publish(m)

    def _pub_status(self, s):
        m = String(); m.data = s
        self.pub_status.publish(m)


# ─── Entry Point ─────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GridManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # DO NOT call rclpy.shutdown() — avoids the "already called" error


if __name__ == '__main__':
    main()