#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        self.get_logger().info("Keyboard teleop started. Use W/A/S/D to move, Q to quit.")

        self.speed = 0.3       # m/s
        self.turn = 1.0        # rad/s

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        twist = Twist()
        while rclpy.ok():
            key = self.get_key()
            if key.lower() == 'w':
                twist.linear.x = self.speed
                twist.angular.z = 0.0
            elif key.lower() == 's':
                twist.linear.x = -self.speed
                twist.angular.z = 0.0
            elif key.lower() == 'd':
                twist.linear.x = 0.0
                twist.angular.z = -self.turn
            elif key.lower() == 'a':
                twist.linear.x = 0.0
                twist.angular.z = self.turn
            elif key.lower() == 'q':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                break
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
