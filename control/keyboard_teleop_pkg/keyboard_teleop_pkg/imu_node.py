import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import adafruit_mpu6050
import board
import busio
import numpy as np

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # Parameters
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)

        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Initialize I2C and MPU6050
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.imu = adafruit_mpu6050.MPU6050(i2c)
            self.get_logger().info("MPU6050 initialized")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MPU6050: {str(e)}")
            raise

        # Create publisher
        self.publisher_ = self.create_publisher(Imu, self.imu_topic, 10)

        # Timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_imu_data)

    def publish_imu_data(self):
        try:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id

            # Read raw data
            accel = np.array(self.imu.acceleration)
            gyro = np.array(self.imu.gyro)

            # Set data
            imu_msg.linear_acceleration.x = accel[0]
            imu_msg.linear_acceleration.y = accel[1]
            imu_msg.linear_acceleration.z = accel[2]

            imu_msg.angular_velocity.x = gyro[0]
            imu_msg.angular_velocity.y = gyro[1]
            imu_msg.angular_velocity.z = gyro[2]

            # Publish
            self.publisher_.publish(imu_msg)

        except Exception as e:
            self.get_logger().error(f"Error reading IMU: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

