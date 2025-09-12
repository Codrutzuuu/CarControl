import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description(): 
    package_dir = get_package_share_directory('keyboard_teleop_pkg')
    config_dir = os.path.join(package_dir, 'config')
    urdf_dir = os.path.join(package_dir, 'urdf')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock if true'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(config_dir, 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file'
        ),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'
        ),

        # Hardware nodes
        Node(
            package='keyboard_teleop_pkg',
            executable='cmd_vel_listener',
            name='cmd_vel_listener',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        Node(
            package='keyboard_teleop_pkg',
            executable='imu_node',
            name='imu_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': 115200, 
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
              
            }],
            output='screen'
        ),
        

        # SLAM and Localization
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                os.path.join(config_dir, 'slam_toolbox_config.yaml'),
                {'use_sim_time': use_sim_time},
                {'publish_tf': True},
                {'tf_buffer_duration': 30.0}
            ],
            output='screen'
        ),

        # Robot State and Transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'robot_description': open(os.path.join(urdf_dir, 'robot.urdf')).read()},
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_base_link_tf',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_base_link_tf',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # EKF for sensor fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[
                os.path.join(config_dir, 'ekf_config.yaml'),
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),

        # Nav2 Core Nodes
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[configured_params],
            remappings=remappings,
            output='screen'
        ),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            parameters=[configured_params],
            remappings=remappings,
            output='screen'
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[configured_params],
            remappings=remappings,
            output='screen'
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[configured_params],
            remappings=remappings,
            output='screen'
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[configured_params],
            remappings=remappings,
            output='screen'
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            parameters=[configured_params],
            remappings=remappings,
            output='screen'
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            parameters=[configured_params],
            remappings=remappings,
            output='screen'
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {
                    'node_names': [
                        'controller_server',
                        'smoother_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'velocity_smoother'
                    ]
                }
            ],
            output='screen'
        ),

        # Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
