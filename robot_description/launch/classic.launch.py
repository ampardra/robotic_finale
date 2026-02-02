import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_description')

    # --- Launch arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # Choose controller: "pid" or "mpc"
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='pid',
        description='Path follower controller type: pid or mpc'
    )

    use_pid = LaunchConfiguration('controller').perform if False else None

    use_pid_arg = DeclareLaunchArgument(
        'use_pid',
        default_value='true',
        description='If true, run PID follower. If false, run MPC follower.'
    )

    # --- PID follower node ---
    pid_follower_node = Node(
        package='robot_description',
        executable='pid_path_follower_node',
        name='pid_path_follower',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),

            # PID follower tuning (optional)
            'map_frame': 'map',
            'base_frame': 'base_link',
            'path_topic': '/global_path',
            'cmd_vel_topic': '/cmd_vel',

            'max_linear_vel': 0.4,
            'max_angular_vel': 1.2,
            'ang_kp': 2.2,
            'ang_ki': 0.0,
            'ang_kd': 0.15,
            'v_nominal': 0.25,
            'v_min': 0.08,
            'slow_down_angle_rad': 0.8,
            'lookahead_distance': 0.6,
            'goal_tolerance': 0.20,
            'control_frequency': 20.0,
            'integral_limit': 1.0,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        condition=IfCondition(LaunchConfiguration('use_pid'))
    )

    # --- MPC follower node ---
    mpc_follower_node = Node(
        package='robot_description',
        executable='mpc_path_follower_node',
        name='mpc_path_follower',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),

            # MPC follower tuning (optional)
            'map_frame': 'map',
            'base_frame': 'base_link',
            'path_topic': '/global_path',
            'cmd_vel_topic': '/cmd_vel',

            'v_max': 0.45,
            'w_max': 1.5,
            'v_ref': 0.25,
            'dt': 0.10,
            'horizon': 12,
            'lookahead_distance': 0.7,
            'goal_tolerance': 0.20,

            'Q_cte': 4.0,
            'Q_heading': 2.0,
            'R_w': 0.2,
            'R_dw': 0.4,
            'R_v': 0.3,
            'w_samples': 11,
            'v_samples': 3,
            'control_frequency': 10.0,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
        condition=UnlessCondition(LaunchConfiguration('use_pid'))
    )

    return LaunchDescription([
        use_sim_time_arg,
        controller_arg,   
        use_pid_arg,      # actual switch (true=PID, false=MPC)
        pid_follower_node,
        mpc_follower_node,
    ])
