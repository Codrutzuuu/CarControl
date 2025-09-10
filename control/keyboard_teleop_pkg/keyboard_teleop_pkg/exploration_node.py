
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import time

class ExplorationNode(Node):
    def __init__(self):
        super().__init__('lidar_avoidance_smart')
        self.get_logger().info("Smart Lidar Avoidance with Arduino Direct Control Started")

        # Configurare serial Arduino
        self.arduino = serial.Serial('/dev/ttyACM0', 57600, timeout=1)

        # Subscriber pe LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Parametri
        self.obstacle_distance = 1.5  # prag obstacol (metri)
        self.forward_speed = 50       # PWM înainte
        self.turn_speed = 40          # PWM viraj
        self.brake_pin_active = True 

        # Stare
        self.state = "FORWARD"
        self.turn_direction = None

    def send_motor_command(self, right, left):

        cmd = f"m {right} {left}\r"
        self.arduino.write(cmd.encode())
        self.get_logger().info(f"Sent to Arduino: {cmd.strip()}")

    def brake(self):
     
        self.send_motor_command(0, 0)

    def scan_callback(self, msg: LaserScan):
        ranges = list(msg.ranges)
        num_points = len(ranges)
        center_index = num_points // 2

       
        sector_width = num_points // 24  # ~15°
        left_sector = ranges[center_index + sector_width : center_index + sector_width*5]
        right_sector = ranges[center_index - sector_width*5 : center_index - sector_width]
        front_sector = ranges[center_index - sector_width : center_index + sector_width]

        # Filtrare valori inf/nan
        def clean(data):
            return [d for d in data if not (d == float('inf') or d != d)]

        left_sector = clean(left_sector)
        right_sector = clean(right_sector)
        front_sector = clean(front_sector)

        # Distan?e minime
        min_front = min(front_sector) if front_sector else float('inf')
        avg_left = sum(left_sector)/len(left_sector) if left_sector else float('inf')
        avg_right = sum(right_sector)/len(right_sector) if right_sector else float('inf')

        if min_front < self.obstacle_distance:
          
            if self.state != "AVOID":
                self.get_logger().warn("Obstacle detected! BRAKING!")
                self.brake()
                time.sleep(0.5) 

              
                if avg_left > avg_right:
                    self.turn_direction = "LEFT"
                else:
                    self.turn_direction = "RIGHT"

                self.get_logger().warn(f"Turning {self.turn_direction}")
                self.state = "AVOID"
        else:
           
            if self.state != "FORWARD":
                self.get_logger().info("Path clear. Moving forward.")
            self.state = "FORWARD"

        self.actuate()

    def actuate(self):
        if self.state == "FORWARD":
            self.send_motor_command(self.forward_speed, self.forward_speed)
        elif self.state == "AVOID":
            if self.turn_direction == "LEFT":
                self.send_motor_command(self.turn_speed, -self.turn_speed)
            else:
                self.send_motor_command(-self.turn_speed, self.turn_speed)

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
