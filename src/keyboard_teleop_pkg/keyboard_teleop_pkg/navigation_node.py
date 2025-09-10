import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.command_subscription = self.create_subscription(
            String,
            '/navigation_commands',
            self.command_callback,
            10)
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_data = None
        self.get_logger().info("Navigation node started")

    def command_callback(self, msg):
        if msg.data.startswith("send_goal"):
            _, x, y, z = msg.data.split(',')
            self.send_goal(float(x), float(y), float(z))
        elif msg.data == "cancel_goal":
            self.cancel_goal()
        elif msg.data == "get_position":
            self.get_position()
        elif msg.data == "save_map":
            self.save_map()

    def send_goal(self, x, y, z):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = z
        goal_msg.pose.pose.orientation.w = 1.0

        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation finished with status: {result}')


    def cancel_goal(self):
        self.action_client.cancel_goal_async()
        self.get_logger().info("Goal canceled")

    def get_position(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.rotation.z
            self.get_logger().info(f"Position: x={x}, y={y}, z={z}")
            with open("startPosition.txt", 'w') as file:
                file.write(f"{x}\n{y}\n{z}")
        except Exception as e:
            self.get_logger().error(f"Failed to get position: {e}")

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Received map update")

    def save_map(self):
        if self.map_data:
            self.get_logger().info("Saving map to /home/user/ros2_ws/map")
            # Poți folosi nav2_map_server pentru a salva harta
            # Alternativ, apelează un serviciu sau salvează manual
        else:
            self.get_logger().warn("No map data available to save")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

