#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')
        self.get_logger().info("OdomBridge started: /odometry/filtered /odom")

        # Subscriber pe /odometry/filtered
        self.sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        # Publisher pe /odom
        self.pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

    def odom_callback(self, msg):
        # Publicã mesajul primit pe /odom
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
