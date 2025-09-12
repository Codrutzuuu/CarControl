import time
import board
import busio
import adafruit_mpu6050
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler
from ahrs.filters import Madgwick 

class ImuNode(Node):
    def __init__(self): 
        super().__init__('imu_node')
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)

        # Ini?ializeazã I2C ?i senzorul
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = adafruit_mpu6050.MPU6050(self.i2c)

        # Calibrare automatã
        self.get_logger().info("Calibrare IMU")
        self.accel_offsets, self.gyro_offsets = self.calibrate_imu(samples=200)
        self.get_logger().info(f"Accel offsets: {self.accel_offsets}")
        self.get_logger().info(f"Gyro offsets: {self.gyro_offsets}")

        # Ini?ializeazã filtrul Madgwick
        self.madgwick = Madgwick()
        self.orientation = [0.0, 0.0, 0.0, 1.0]  # quaternion ini?ial

        # Timer 100 Hz
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("IMU node started")

    def calibrate_imu(self, samples=200):
        accel_offsets = [0.0, 0.0, 0.0]
        gyro_offsets = [0.0, 0.0, 0.0]
        for i in range(samples):
            accel = self.imu.acceleration
            gyro = self.imu.gyro
            for j in range(3):
                accel_offsets[j] += accel[j]
                gyro_offsets[j] += gyro[j]
            time.sleep(0.01)  # 100 Hz

        accel_offsets = [x / samples for x in accel_offsets]
        gyro_offsets = [x / samples for x in gyro_offsets]
        return accel_offsets, gyro_offsets

    def timer_callback(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Cite?te date brute
        accel = self.imu.acceleration
        gyro = self.imu.gyro

        # Aplicã offset-uri
        accel = [accel[i] - self.accel_offsets[i] for i in range(3)]
        gyro = [gyro[i] - self.gyro_offsets[i] for i in range(3)]

        # Madgwick update (gyro în rad/s, accel în m/s2)
        self.orientation = self.madgwick.updateIMU(
            self.orientation,
            gyr=[gyro[0], gyro[1], gyro[2]],
            acc=[accel[0], accel[1], accel[2]]
        )

        # Seteazã orientarea în mesaj
        imu_msg.orientation.x = self.orientation[0]
        imu_msg.orientation.y = self.orientation[1]
        imu_msg.orientation.z = self.orientation[2]
        imu_msg.orientation.w = self.orientation[3]

        # Seteazã accelera?ia liniarã
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]

        # Seteazã viteza unghiularã (rad/s)
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]

        # Publicã mesajul
        self.imu_publisher.publish(imu_msg)

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
