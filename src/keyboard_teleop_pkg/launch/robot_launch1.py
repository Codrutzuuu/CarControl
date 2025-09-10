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

    # Remappings for Nav2 TF
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Param substitutions for Nav2
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Load URDF via xacro from diffdrive_arduino
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("diffdrive_arduino"), "urdf", "diffbot.urdf.xacro"
        ])
    ])
    robot_description = {"robot_description": robot_description_content}

    # Controller config
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("diffdrive_arduino"),
        "config",
        "diffbot_controllers.yaml"
    ])

    # Nodes for ros2_control
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both"
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output="both"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"]
    )

    # Delay robot controller start until joint_state_broadcaster is ready
    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner]
        )
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

    # SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            os.path.join(config_dir, 'slam_toolbox_config.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # EKF Localization
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            os.path.join(config_dir, 'ekf_config.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Static transforms for laser and IMU
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_to_base_link_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_base_link_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    rviz2=  Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )

    # Nav2 Core Nodes
    nav2_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=remappings
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=remappings
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=remappings
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=remappings
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': [
                    'controller_server',
                    'planner_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]}
            ]
        )
    ]


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

        # ros2_control + robot_state_publisher
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner,

        # SLAM + EKF
        sllidar_ros2,
        slam_node,
        ekf_node,

        # Static transforms
        laser_tf,
        imu_tf,

        # Nav2 stack
        *nav2_nodes,
        rviz2
    ])
