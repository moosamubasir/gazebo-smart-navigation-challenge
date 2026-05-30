import math
import random
from pathlib import Path
from typing import List, Tuple

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node


Cell = Tuple[int, int]


class BonusRandomizer(Node):
    def __init__(self):
        super().__init__('bonus_randomizer')
        self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')
        self.start: Cell = (-2, -2)
        self.end: Cell = (2, 2)
        self.obstacles = {(-2, 0), (-1, -1), (0, 1), (1, 1), (2, -1)}
        self.bonus_names = ['bonus_1', 'bonus_2', 'bonus_3']
        self.last_locations_file = Path.home() / '.ros' / 'gazebo_tutorial_last_bonus_locations.txt'

    def randomize(self):
        random.seed()
        self.set_state_client.wait_for_service(timeout_sec=8.0)
        cells = self.available_cells()
        selected_cells = self.pick_new_cells(cells)

        for name, cell in zip(self.bonus_names, selected_cells):
            self.move_bonus(name, cell)

        self.save_last_locations(selected_cells)
        self.get_logger().info(
            'Bonus locations this round: '
            + ', '.join(f'{name}={cell}' for name, cell in zip(self.bonus_names, selected_cells))
        )

    def pick_new_cells(self, cells: List[Cell]) -> List[Cell]:
        previous = self.load_last_locations()
        for _ in range(100):
            selected = random.sample(cells, len(self.bonus_names))
            if all(previous.get(name) != cell for name, cell in zip(self.bonus_names, selected)):
                return selected
        return random.sample(cells, len(self.bonus_names))

    def load_last_locations(self) -> dict:
        if not self.last_locations_file.exists():
            return {}
        previous = {}
        for line in self.last_locations_file.read_text().splitlines():
            parts = line.split()
            if len(parts) != 3:
                continue
            name, x, y = parts
            previous[name] = (int(x), int(y))
        return previous

    def save_last_locations(self, cells: List[Cell]):
        self.last_locations_file.parent.mkdir(parents=True, exist_ok=True)
        lines = [
            f'{name} {cell[0]} {cell[1]}'
            for name, cell in zip(self.bonus_names, cells)
        ]
        self.last_locations_file.write_text('\n'.join(lines) + '\n')

    def available_cells(self) -> List[Cell]:
        blocked = set(self.obstacles)
        blocked.add(self.start)
        blocked.add(self.end)
        return [
            (x, y)
            for x in range(-2, 3)
            for y in range(-2, 3)
            if (x, y) not in blocked
        ]

    def move_bonus(self, name: str, cell: Cell):
        request = SetEntityState.Request()
        state = EntityState()
        state.name = name
        state.reference_frame = 'world'
        state.pose.position.x = float(cell[0])
        state.pose.position.y = float(cell[1])
        state.pose.position.z = 0.045
        state.pose.orientation.w = 1.0
        state.pose.orientation.z = math.sin(0.0)
        request.state = state
        future = self.set_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = BonusRandomizer()
    try:
        node.randomize()
        rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()
