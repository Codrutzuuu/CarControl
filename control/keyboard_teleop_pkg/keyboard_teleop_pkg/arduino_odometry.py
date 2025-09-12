#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import serial
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class ArduinoOdometryNode(Node):
    def __init__(self):
        super().__init__('arduino_odometry_node')
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def timer_callback(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('ascii').strip()
            if line.startswith('<ODOM,'):
                try:
                    _, x, y, theta = map(float, line[5:-1].split(','))
                    self.x = x
                    self.y = y
                    self.theta = theta

                    odom = Odometry()
                    odom.header.stamp = self.get_clock().now().to_msg()
                    odom.header.frame_id = "odom"
                    odom.child_frame_id = "base_link"
                    odom.pose.pose.position.x = self.x
                    odom.pose.pose.position.y = self.y
                    q = self.quaternion_from_euler(0, 0, self.theta)
                    odom.pose.pose.orientation.x = q[0]
                    odom.pose.pose.orientation.y = q[1]
                    odom.pose.pose.orientation.z = q[2]
                    odom.pose.pose.orientation.w = q[3]
                    self.odom_publisher.publish(odom)

                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = "odom"
                    t.child_frame_id = "base_link"
                    t.transform.translation.x = self.x
                    t.transform.translation.y = self.y
                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]
                    self.tf_broadcaster.sendTransform(t)
                except ValueError:
                    self.get_logger().error(f"Invalid odom data: {line}")

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = [0.0] * 4
        q[0] = cr * cp * sy - sr * sp * cy
        q[1] = sr * cp * cy + cr * sp * sy
        q[2] = sr * cp * sy - cr * sp * cy
        q[3] = cr * cp * cy + sr * sp * sy
        return q

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
