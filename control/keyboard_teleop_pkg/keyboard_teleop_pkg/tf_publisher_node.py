import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion, Twist  # Adaugă Twist aici
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import math

class TfPublisherNode(Node):
    def __init__(self):
        super().__init__('tf_publisher_node')
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Twist,  # Folosește direct Twist
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.get_logger().info("TF publisher node started")

    def cmd_vel_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        v = math.sqrt(msg.linear.x**2 + msg.linear.y**2)
        w = msg.angular.z
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        odom.twist.twist = msg
        self.odom_publisher.publish(odom)

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TfPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
