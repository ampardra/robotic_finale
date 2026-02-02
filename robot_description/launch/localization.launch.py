import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path


def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_description')
    map_yaml_file = os.path.join(bringup_dir, 'maps', 'map.yaml')
    amcl_config_file = os.path.join(bringup_dir, 'config', 'amcl.yaml')
    world_file = os.path.join(bringup_dir, 'world', 'depot.sdf')
    urdf_file = os.path.join(bringup_dir, 'src', 'description', 'robot.urdf')
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'config.rviz')
    gz_bridge_config = os.path.join(bringup_dir, 'config', 'gz_bridge.yaml')
    vo_config_file = os.path.join(bringup_dir, 'config', 'rtabmap_slam.yaml')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=map_yaml_file,
        description='Full path to map yaml file to load'
    )

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(bringup_dir, 'world'),
            str(Path(bringup_dir).parent.resolve())
        ])
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': robot_desc}
        ]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={"gz_args": ["-r -v 4 ", world_file]}.items(),
    )

    bridge_topics = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': gz_bridge_config,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    bridge_service_control = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/depot/control@ros_gz_interfaces/srv/ControlWorld'
        ],
        output='screen'
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "robot",
            "-topic", "/robot_description",
            "-x", "0",
            "-y", "0",
            "-z", "0.9",
        ],
        output="screen",
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    frame_id_converter_node = Node(
        package='robot_description',
        executable='frame_id_converter_node',
        name='frame_id_converter_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    ekf_diff_imu_node = Node(
        package='robot_description',
        executable='ekf_diff_imu_node',
        name='ekf_diff_imu_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'yaml_filename': LaunchConfiguration('map')}
        ],
        remappings=[('/tf', 'tf'),
                   ('/tf_static', 'tf_static')]
    )

    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='',
        output='screen',
        parameters=[
            amcl_config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[('/tf', 'tf'),
                   ('/tf_static', 'tf_static'),
                   ('scan', '/scan_pc')
                ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {
                # 'target_frame': 'base_link',
                'transform_tolerance': 0.1,
                'min_height': -0.2,
                'max_height': 0.2,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.00872665,
                'scan_time': 0.1,
                'range_min': 0.12,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }
        ],
        remappings=[
            ('cloud_in', '/gz_lidar/points'),  # <-- replace with your actual ROS PointCloud2 topic
            ('scan', '/scan_pc')
        ]
    )


    rtabmap_vo_node = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rtabmap_visual_odometry",
        output="screen",
        parameters=[
            vo_config_file,
            {
                "use_sim_time": True,
                "publish_tf": False
            }
        ],
        remappings=[
            ("/rgb/image", "/zed/zed_node/left/image_rect_color"),
            ("/depth/image", "/zed/zed_node/depth/depth_registered"),
            ("/rgb/camera_info", "/zed/zed_node/left/camera_info"),
            ("/odom", "/vo/odom")

        ]
    )

    tf_fix_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gazebo_frame_fix',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'robot/base_link/rplidar_c1_sensor']
    )

    return LaunchDescription([
        rtabmap_vo_node,
        use_sim_time_arg,
        map_yaml_arg,
        gz_resource_path,
        robot_state_publisher,
        gz_sim,
        bridge_topics,
        bridge_service_control,
        spawn_entity,
        rviz_node,
        frame_id_converter_node,
        ekf_diff_imu_node,
        pointcloud_to_laserscan_node,
        map_server_node,
        amcl_node,
        lifecycle_manager,
        tf_fix_node,

    ])