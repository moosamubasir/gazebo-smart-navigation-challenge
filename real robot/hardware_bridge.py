#!/usr/bin/env python3
"""
hardware_bridge.py — Bridge between ROS 2 /cmd_vel and Arduino serial.

────────────────────────────────────────────────────────────────────────────────
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial, threading, time

# ── CALIBRATION ──────────────────────────────────────────────────────────────
TURN_DURATION_MS = 700   # ms for exact 90-degree turn
FWD_DURATION_MS  = 1000  # ms for one full 25cm grid cell
# ─────────────────────────────────────────────────────────────────────────────


class HardwareBridge(Node):
    def __init__(self):
        super().__init__('hardware_bridge')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=10.0)
        time.sleep(2.0)  # Wait for Arduino to boot before sending commands!
        self.busy = threading.Event()

        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 10)
        self.done_pub = self.create_publisher(String, '/robot/move_done', 10)

        self.get_logger().info(
            f'Hardware Bridge Ready. FWD={FWD_DURATION_MS}ms, TURN={TURN_DURATION_MS}ms')

    def cb_cmd(self, msg):
        if self.busy.is_set():
            return

        lx, az = msg.linear.x, msg.angular.z

        if abs(lx) > 0.01:
            cmd = f'F{FWD_DURATION_MS}\n'
        elif az > 0.1:    # Left turn
            cmd = f'L{TURN_DURATION_MS}\n'
        elif az < -0.1:   # Right turn
            cmd = f'R{TURN_DURATION_MS}\n'
        else:
            return  # zero Twist → stop (already stopped by Arduino timeout)

        threading.Thread(target=self.send, args=(cmd,), daemon=True).start()

    def send(self, cmd):
        self.busy.set()
        self.get_logger().info(f'→ Arduino: {cmd.strip()}')
        try:
            self.ser.write(cmd.encode())
            # Wait for Arduino to respond with DONE
            while True:
                line = self.ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                self.get_logger().info(f'← Arduino: {line}')
                if line == 'DONE':
                    break
        except Exception as e:
            self.get_logger().error(f'Serial error: {e}')
        finally:
            self.busy.clear()
            # Notify autonomous_robot.py that this move is complete
            msg = String()
            msg.data = 'DONE'
            self.done_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridge()
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
