#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.get_logger().info("CmdVelBridge started: /cmd_vel /diffbot_base_controller/cmd_vel_unstamped")
        
        # Subscriber pe /cmd_vel
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher pe /cmd_vel_unstamped
        self.pub = self.create_publisher(
            Twist,
            '/diffbot_base_controller/cmd_vel_unstamped',
            10
        )

    def cmd_vel_callback(self, msg):
        # Publicã mesajul primit pe cmd_vel_unstamped
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
