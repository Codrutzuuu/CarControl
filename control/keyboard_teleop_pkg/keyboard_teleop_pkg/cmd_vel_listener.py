#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import serial
import math
import tf2_ros
import tf_transformations
from rcl_interfaces.msg import ParameterDescriptor
import threading

class CmdVelListener(Node):
    def __init__(self):
        super().__init__('cmd_vel_listener')

        # Enhanced parameter declarations with validation
        params = [
            ('serial_port', '/dev/ttyACM0', 'Serial port for Arduino'),
            ('baud_rate', 57600, 'Serial communication baud rate'),
            ('wheel_radius', 0.14, 'Wheel radius in meters'),
            ('wheel_base', 0.58, 'Distance between wheel centers'),
            ('ticks_per_rev', 94, 'Encoder ticks per revolution'),
            ('max_encoder_delta', 1000, 'Max acceptable encoder delta'),
            ('max_linear_speed', 0.5, 'Maximum linear speed (m/s)'),
            ('safety_timeout', 1.0, 'Motor safety timeout (seconds)')
        ]
        
        for name, default, desc in params:
            self.declare_parameter(name, default, ParameterDescriptor(description=desc))
        
        # Load and validate parameters
        self.WHEEL_RADIUS = self.validate_param('wheel_radius', 0.01, 0.5)
        self.WHEEL_BASE = self.validate_param('wheel_base', 0.1, 2.0)
        self.TICKS_PER_REV = self.validate_param('ticks_per_rev', 1, 10000)
        self.MAX_ENCODER_DELTA = self.validate_param('max_encoder_delta', 10, 10000)
        self.MAX_LINEAR_SPEED = self.validate_param('max_linear_speed', 0.1, 5.0)
        self.SAFETY_TIMEOUT = self.validate_param('safety_timeout', 0.1, 10.0)

        # Serial initialization
        self.init_serial()
        
        # State management
        self.lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_encoders = (0, 0)
        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()
        
        # Motor control state
        self.last_pwmR = 0
        self.last_pwmL = 0
        
        # ROS interfaces
        self.init_ros_components()
        
        # Low-pass filters
        self.linear_filter = self.LowPassFilter(0.2)
        self.angular_filter = self.LowPassFilter(0.2)
        
        self.get_logger().info("Enhanced CmdVelListener initialized")

    def validate_param(self, name, min_val, max_val):
        value = self.get_parameter(name).value
        if not (min_val <= value <= max_val):
            self.get_logger().fatal(f"Invalid {name}: {value} (valid: {min_val}-{max_val})")
            raise ValueError(f"Invalid {name}")
        return value

    def init_serial(self):
        try:
            self.serial = serial.Serial(
                self.get_parameter('serial_port').value,
                self.get_parameter('baud_rate').value,
                timeout=0.1
            )
            self.get_logger().info(f"Connected to {self.serial.port}")
        except serial.SerialException as e:
            self.get_logger().fatal(f"Serial error: {str(e)}")
            raise

    def init_ros_components(self):
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
            
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.create_timer(0.02, self.odometry_update)  # 50Hz
        self.create_timer(0.1, self.safety_check)  # 10Hz

    def cmd_vel_callback(self, msg):
        # Input validation
        if abs(msg.linear.x) > self.MAX_LINEAR_SPEED or abs(msg.angular.z) > (2*self.MAX_LINEAR_SPEED/self.WHEEL_BASE):
            self.get_logger().warn("Dangerous command rejected!")
            return
            
        # Differential drive calculations
        v_left = msg.linear.x - (msg.angular.z * self.WHEEL_BASE / 2)
        v_right = msg.linear.x + (msg.angular.z * self.WHEEL_BASE / 2)
        
        # Convert to PWM with rate limiting
        pwmR = int((v_right / self.MAX_LINEAR_SPEED) * 255)
        pwmL = int((v_left / self.MAX_LINEAR_SPEED) * 255)
        
        # Rate limiting
        pwmR = sorted([self.last_pwmR-15, pwmR, self.last_pwmR+15])[1]
        pwmL = sorted([self.last_pwmL-15, pwmL, self.last_pwmL+15])[1]
        
        # Direction handling
        dir_code = 0
        if v_right >= 0 and v_left >= 0:
            dir_code = 1
        elif v_right < 0 and v_left < 0:
            dir_code = 0
        elif v_right < 0 and v_left > 0:
            dir_code = 2
        else:
            dir_code = 3
            
        # Send command
        cmd = f"<{pwmR},{pwmL},{dir_code}>"
        self.serial.write(cmd.encode('ascii'))
        self.last_cmd_time = self.get_clock().now()
        self.last_pwmR = pwmR
        self.last_pwmL = pwmL

    def odometry_update(self):
        with self.lock:
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            self.last_time = now
            
            # Read and process encoders
            enc_r, enc_l = self.read_encoders()
            delta_r = self.calc_delta(enc_r, self.last_encoders[0])
            delta_l = self.calc_delta(enc_l, self.last_encoders[1])
            
            # Update stored values
            self.last_encoders = (enc_r, enc_l)
            
            # Validate encoder data
            if abs(delta_r) > self.MAX_ENCODER_DELTA or abs(delta_l) > self.MAX_ENCODER_DELTA:
                self.get_logger().error("Encoder jump detected! Resetting odometry")
                self.reset_odometry()
                return
                
            # Calculate distances
            dist_r = self.ticks_to_meters(delta_r)
            dist_l = self.ticks_to_meters(delta_l)
            
            # Compute velocities with filtering
            v_r = self.linear_filter.update(dist_r / dt) if dt > 0 else 0.0
            v_l = self.linear_filter.update(dist_l / dt) if dt > 0 else 0.0
            
            # Odometry calculations
            linear_vel = (v_r + v_l) / 2.0
            angular_vel = (v_r - v_l) / self.WHEEL_BASE
            
            # Update pose using improved integration
            self.update_pose(linear_vel, angular_vel, dt)
            
            # Publish odometry data
            self.publish_odom(now, linear_vel, angular_vel)
            self.publish_tf(now)

    def update_pose(self, linear_vel, angular_vel, dt):
        # RK2 integration
        if abs(angular_vel) < 1e-6:
            self.x += linear_vel * math.cos(self.theta) * dt
            self.y += linear_vel * math.sin(self.theta) * dt
        else:
            radius = linear_vel / angular_vel
            d_theta = angular_vel * dt
            self.x += radius * (math.sin(self.theta + d_theta) - math.sin(self.theta))
            self.y += -radius * (math.cos(self.theta + d_theta) - math.cos(self.theta))
            self.theta += d_theta
            
        # Angle normalization
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

    def publish_odom(self, timestamp, linear_vel, angular_vel):
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Velocity
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel
        
        # Dynamic covariance
        odom.pose.covariance = self.dynamic_covariance(linear_vel, angular_vel)
        
        self.odom_pub.publish(odom)

    def dynamic_covariance(self, linear, angular):
        trans_var = 0.01 + abs(linear) * 0.1
        rot_var = 0.01 + abs(angular) * 0.2
        return [
            trans_var, 0, 0, 0, 0, 0,
            0, trans_var, 0, 0, 0, 0,
            0, 0, 0.0001, 0, 0, 0,
            0, 0, 0, 0.0001, 0, 0,
            0, 0, 0, 0, 0.0001, 0,
            0, 0, 0, 0, 0, rot_var
        ]

    def publish_tf(self, timestamp):
        tf = TransformStamped()
        tf.header.stamp = timestamp.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf)

    def safety_check(self):
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds > 1e9 * self.SAFETY_TIMEOUT:
            self.serial.write(b"<0,0,0>")
            self.last_pwmR = 0
            self.last_pwmL = 0

    def read_encoders(self):
        self.serial.write(b"get_encoders\n")
        line = self.serial.readline().decode().strip()
        if line.startswith("ENC:"):
            try:
                return tuple(map(int, line[4:].split(',')))
            except ValueError:
                self.get_logger().warn("Invalid encoder data")
                return self.last_encoders
        return self.last_encoders

    def calc_delta(self, current, last):
        MAX_ENCODER = 4294967295
        delta = current - last
        if delta < -MAX_ENCODER/2:
            delta += MAX_ENCODER + 1
        elif delta > MAX_ENCODER/2:
            delta -= MAX_ENCODER + 1
        return delta

    def ticks_to_meters(self, ticks):
        return (ticks / self.TICKS_PER_REV) * (2 * math.pi * self.WHEEL_RADIUS)

    def reset_odometry(self):
        with self.lock:
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.last_encoders = (0, 0)

    class LowPassFilter:
        def __init__(self, time_constant):
            self.alpha = 1.0 / (1.0 + time_constant)
            self.value = 0.0
            
        def update(self, new_val):
            self.value = self.alpha * new_val + (1 - self.alpha) * self.value
            return self.value

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CmdVelListener()
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().fatal(f"Node failure: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
