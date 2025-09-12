import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Package directories
    package_dir = get_package_share_directory('keyboard_teleop_pkg')
    config_dir = os.path.join(package_dir, 'config')

    # Launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')


    # Param substitutions for Nav2
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local
    }


    # Nodes for ros2_control
    exploration_node = Node(
        package="keyboard_teleop_pkg",
        executable="exploration_node",
        name='exploration_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )


    sllidar_ros2 = Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200, 
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
                'fixed_frame': 'base_link'
            },  {'use_sim_time': use_sim_time}],
            output='screen'
        )



    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(config_dir, 'nav2_params.yaml'),
                              description='Full path to the ROS2 parameters file to use'),
        DeclareLaunchArgument('default_bt_xml_filename',
                              default_value=os.path.join(
                                  get_package_share_directory('nav2_bt_navigator'),
                                  'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'),
                              description='Full path to the behavior tree xml file to use'),
        DeclareLaunchArgument('map_subscribe_transient_local', default_value='true',
                              description='Whether to set the map subscriber QoS to transient local'),



        # SLAM + EKF
        exploration_node,
        sllidar_ros2,

    ])
