import heapq
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import DeleteEntity, SetEntityState
from geometry_msgs.msg import Twist
from rclpy.node import Node


Cell = Tuple[int, int]


class CollectorController(Node):
    def __init__(self):
        super().__init__('collector_controller')

        self.robot_name = 'collector_bot'
        self.bonus_names = ['bonus_1', 'bonus_2', 'bonus_3']
        self.start: Cell = (-2, -2)
        self.goal: Cell = (2, 2)
        self.fixed_obstacles = {(-2, 0), (-1, -1), (0, 1), (1, 1)}
        self.obstacles = set(self.fixed_obstacles)
        self.moving_obstacle_name = 'obstacle_5'
        self.moving_obstacle_start: Optional[Cell] = None
        self.moving_obstacle_goal: Optional[Cell] = None
        self.moving_obstacle_step = 0
        self.moving_obstacle_steps = 18
        self.moving_obstacle_deployed = False
        self.moving_obstacle_finished = False

        self.pose: Optional[Tuple[float, float, float]] = None
        self.bonuses: Dict[str, Cell] = {}
        self.remaining_bonuses = set(self.bonus_names)
        self.mission_targets: List[str] = []
        self.current_target_name: Optional[str] = None
        self.path: List[Cell] = []
        self.target_index = 0
        self.route_ready = False
        self.finished = False
        self.speed = 0.45

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.create_subscription(ModelStates, '/model_states', self.model_states_callback, 10)

        self.set_state_client.wait_for_service(timeout_sec=5.0)
        self.delete_client.wait_for_service(timeout_sec=5.0)
        self.timer = self.create_timer(0.08, self.control_loop)

        self.get_logger().info('Waiting for bonus locations...')

    def model_states_callback(self, msg: ModelStates):
        if self.robot_name not in msg.name:
            return

        robot_index = msg.name.index(self.robot_name)
        robot_pose = msg.pose[robot_index]
        yaw = self.yaw_from_quaternion(robot_pose.orientation)
        self.pose = (robot_pose.position.x, robot_pose.position.y, yaw)

        if self.route_ready:
            return

        for bonus_name in self.bonus_names:
            if bonus_name in msg.name:
                bonus_index = msg.name.index(bonus_name)
                bonus_pose = msg.pose[bonus_index]
                self.bonuses[bonus_name] = self.world_to_cell(
                    bonus_pose.position.x,
                    bonus_pose.position.y,
                )

        if all(name in self.bonuses for name in self.bonus_names):
            self.mission_targets = self.order_bonus_targets()
            self.current_target_name = self.mission_targets[0]
            self.path = self.path_to_current_target()
            self.target_index = 1
            self.route_ready = True
            self.get_logger().info(f'Bonus locations: {self.bonuses}')
            self.get_logger().info(f'Collection order: {self.mission_targets}')
            self.get_logger().info(f'Route to {self.current_target_name}: {self.path}')

    def order_bonus_targets(self) -> List[str]:
        current = self.start
        unvisited = set(self.bonus_names)
        ordered_targets = []

        while unvisited:
            choices = []
            for bonus_name in unvisited:
                path = self.astar(current, self.bonuses[bonus_name])
                if path:
                    choices.append((len(path), bonus_name, path))

            if not choices:
                self.get_logger().error('Could not find a path to all bonus points.')
                return ordered_targets

            _, bonus_name, path = min(choices)
            ordered_targets.append(bonus_name)
            current = self.bonuses[bonus_name]
            unvisited.remove(bonus_name)

        ordered_targets.append('goal')
        return ordered_targets

    def path_to_current_target(self) -> List[Cell]:
        current = self.world_to_cell(self.pose[0], self.pose[1]) if self.pose else self.start
        target = self.goal if self.current_target_name == 'goal' else self.bonuses[self.current_target_name]
        path = self.astar(current, target)
        if not path:
            self.get_logger().error(f'Could not find path from {current} to {self.current_target_name}.')
            return [current]
        return path

    def astar(self, start: Cell, goal: Cell) -> List[Cell]:
        open_set = [(0, start)]
        came_from: Dict[Cell, Cell] = {}
        cost_so_far = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.neighbors(current):
                new_cost = cost_so_far[current] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.manhattan(neighbor, goal)
                    heapq.heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = current

        return []

    def neighbors(self, cell: Cell) -> List[Cell]:
        x, y = cell
        candidates = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        return [
            candidate
            for candidate in candidates
            if -2 <= candidate[0] <= 2
            and -2 <= candidate[1] <= 2
            and candidate not in self.obstacles
        ]

    def update_moving_obstacle(self):
        if not self.moving_obstacle_deployed:
            self.deploy_moving_obstacle()
            return

        if self.moving_obstacle_finished or self.moving_obstacle_start is None:
            return

        self.moving_obstacle_step += 1
        ratio = min(1.0, self.moving_obstacle_step / self.moving_obstacle_steps)
        start_x, start_y = self.cell_to_world(self.moving_obstacle_start)
        goal_x, goal_y = self.cell_to_world(self.moving_obstacle_goal)
        x = start_x + (goal_x - start_x) * ratio
        y = start_y + (goal_y - start_y) * ratio
        self.set_entity_pose(self.moving_obstacle_name, x, y, 0.25, 0.0)

        if ratio >= 1.0:
            self.moving_obstacle_finished = True
            self.get_logger().info(
                f'{self.moving_obstacle_name} crossed into path cell '
                f'{self.moving_obstacle_goal}; robot is avoiding it.'
            )

    def deploy_moving_obstacle(self):
        obstacle_start, obstacle_goal = self.choose_moving_obstacle_cells()
        if obstacle_start is None or obstacle_goal is None:
            self.moving_obstacle_deployed = True
            self.moving_obstacle_finished = True
            self.get_logger().warn('No safe route cell found for moving obstacle demo.')
            return

        self.moving_obstacle_start = obstacle_start
        self.moving_obstacle_goal = obstacle_goal
        self.obstacles = set(self.fixed_obstacles)
        self.obstacles.add(obstacle_goal)
        self.moving_obstacle_deployed = True

        start_x, start_y = self.cell_to_world(obstacle_start)
        self.set_entity_pose(self.moving_obstacle_name, start_x, start_y, 0.25, 0.0)
        self.path = self.path_to_current_target()
        self.target_index = 1 if len(self.path) > 1 else len(self.path)
        self.get_logger().info(
            f'{self.moving_obstacle_name} moves one grid from {obstacle_start} '
            f'to {obstacle_goal}, crossing the original path. New route: {self.path}'
        )

    def choose_moving_obstacle_cells(self) -> Tuple[Optional[Cell], Optional[Cell]]:
        protected = set(self.bonuses.values())
        protected.add(self.start)
        protected.add(self.goal)
        current = self.world_to_cell(self.pose[0], self.pose[1]) if self.pose else self.start
        target = self.goal if self.current_target_name == 'goal' else self.bonuses[self.current_target_name]

        for route_cell in self.path[2:-1]:
            if route_cell in protected or route_cell in self.fixed_obstacles:
                continue
            test_obstacles = set(self.fixed_obstacles)
            test_obstacles.add(route_cell)
            old_obstacles = self.obstacles
            self.obstacles = test_obstacles
            alternate_path = self.astar(current, target)
            self.obstacles = old_obstacles
            if not alternate_path:
                continue
            for start_cell in self.neighbor_cells(route_cell):
                if start_cell in protected:
                    continue
                if start_cell in self.fixed_obstacles or start_cell in self.path:
                    continue
                return start_cell, route_cell

        return None, None

    @staticmethod
    def neighbor_cells(cell: Cell) -> List[Cell]:
        x, y = cell
        candidates = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        return [
            candidate
            for candidate in candidates
            if -2 <= candidate[0] <= 2 and -2 <= candidate[1] <= 2
        ]

    def control_loop(self):
        if self.finished or self.pose is None or not self.route_ready:
            return

        self.update_moving_obstacle()
        self.collect_bonus_if_close()

        if self.target_index >= len(self.path):
            self.handle_target_reached()
            return

        target_x, target_y = self.cell_to_world(self.path[self.target_index])
        x, y, yaw = self.pose
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)

        if distance < 0.15:
            self.set_robot_pose(target_x, target_y, yaw)
            self.target_index += 1
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(desired_yaw - yaw)
        step = min(self.speed * 0.08, distance)
        next_x = x + math.cos(desired_yaw) * step
        next_y = y + math.sin(desired_yaw) * step
        next_yaw = yaw + max(-0.12, min(0.12, yaw_error))
        self.set_robot_pose(next_x, next_y, next_yaw)

    def collect_bonus_if_close(self):
        x, y, _ = self.pose
        for bonus_name in list(self.remaining_bonuses):
            bonus_x, bonus_y = self.cell_to_world(self.bonuses[bonus_name])
            if math.hypot(bonus_x - x, bonus_y - y) <= 0.35:
                self.hide_bonus_now(bonus_name)
                self.delete_bonus(bonus_name)
                self.remaining_bonuses.remove(bonus_name)
                self.get_logger().info(
                    f'Collected {bonus_name}. Remaining: {len(self.remaining_bonuses)}'
                )

    def handle_target_reached(self):
        if self.current_target_name in self.remaining_bonuses:
            self.collect_required_bonus(self.current_target_name)
            return

        if self.current_target_name == 'goal':
            if self.remaining_bonuses:
                self.get_logger().warn(
                    f'Cannot finish yet. Still missing: {sorted(self.remaining_bonuses)}'
                )
                self.advance_to_next_bonus()
                return
            self.stop()
            self.finished = True
            self.get_logger().info('Finished: all bonus points collected and red goal reached.')
            return

        self.advance_to_next_target()

    def collect_required_bonus(self, bonus_name: str):
        if bonus_name not in self.remaining_bonuses:
            self.advance_to_next_target()
            return

        self.hide_bonus_now(bonus_name)
        self.delete_bonus(bonus_name)
        self.remaining_bonuses.remove(bonus_name)
        self.get_logger().info(f'Collected {bonus_name}. Remaining: {len(self.remaining_bonuses)}')
        self.advance_to_next_target()

    def advance_to_next_bonus(self):
        if not self.remaining_bonuses:
            self.current_target_name = 'goal'
        else:
            current = self.world_to_cell(self.pose[0], self.pose[1])
            self.current_target_name = min(
                self.remaining_bonuses,
                key=lambda name: self.manhattan(current, self.bonuses[name]),
            )
        self.path = self.path_to_current_target()
        self.target_index = 1 if len(self.path) > 1 else len(self.path)
        self.get_logger().info(f'Next target: {self.current_target_name}; route: {self.path}')

    def advance_to_next_target(self):
        if self.current_target_name in self.mission_targets:
            index = self.mission_targets.index(self.current_target_name)
            next_index = index + 1
        else:
            next_index = len(self.mission_targets)

        while next_index < len(self.mission_targets):
            candidate = self.mission_targets[next_index]
            if candidate == 'goal' or candidate in self.remaining_bonuses:
                self.current_target_name = candidate
                self.path = self.path_to_current_target()
                self.target_index = 1 if len(self.path) > 1 else len(self.path)
                self.get_logger().info(f'Next target: {self.current_target_name}; route: {self.path}')
                return
            next_index += 1

        self.current_target_name = 'goal'
        self.path = self.path_to_current_target()
        self.target_index = 1 if len(self.path) > 1 else len(self.path)
        self.get_logger().info(f'Next target: goal; route: {self.path}')

    def hide_bonus_now(self, bonus_name: str):
        bonus_x, bonus_y = self.cell_to_world(self.bonuses[bonus_name])
        self.set_entity_pose(bonus_name, bonus_x, bonus_y, -5.0, 0.0)

    def delete_bonus(self, bonus_name: str):
        if not self.delete_client.service_is_ready():
            return
        request = DeleteEntity.Request()
        request.name = bonus_name
        self.delete_client.call_async(request)

    def set_robot_pose(self, x: float, y: float, yaw: float):
        self.set_entity_pose(self.robot_name, x, y, 0.09, yaw)

    def set_entity_pose(self, name: str, x: float, y: float, z: float, yaw: float):
        if not self.set_state_client.service_is_ready():
            return

        request = SetEntityState.Request()
        state = EntityState()
        state.name = name
        state.reference_frame = 'world'
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.z = math.sin(yaw / 2.0)
        state.pose.orientation.w = math.cos(yaw / 2.0)
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.angular.z = 0.0
        request.state = state
        self.set_state_client.call_async(request)

    def stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def cell_to_world(cell: Cell) -> Tuple[float, float]:
        return float(cell[0]), float(cell[1])

    @staticmethod
    def world_to_cell(x: float, y: float) -> Cell:
        return int(round(x)), int(round(y))

    @staticmethod
    def manhattan(a: Cell, b: Cell) -> int:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    @staticmethod
    def reconstruct_path(came_from: Dict[Cell, Cell], current: Cell) -> List[Cell]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    @staticmethod
    def yaw_from_quaternion(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = CollectorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
