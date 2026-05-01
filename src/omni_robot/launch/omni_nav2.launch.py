import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_omni = get_package_share_directory('omni_robot')
    params_file = os.path.join(pkg_omni, 'config', 'omni_nav2_params.yaml')
    slam_params_file = os.path.join(pkg_omni, 'config', 'slam.yaml')

    declare_use_composition = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='If True, run Nav2 in a single component container')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': 'True',
            'map': '',
            'use_sim_time': 'True',
            'params_file': params_file,
            'slam_params_file': slam_params_file,
            'use_composition': LaunchConfiguration('use_composition'),
            'autostart': 'True',
        }.items(),
    )

    cmd_vel_bridge = Node(
        package='omni_robot',
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        output='screen',
    )

    scan_bridge = Node(
        package='omni_robot',
        executable='scan_bridge',
        name='scan_bridge',
        output='screen',
    )

    odom_tf_bridge = Node(
        package='omni_robot',
        executable='odom_tf_bridge',
        name='odom_tf_bridge',
        output='screen',
    )

    target_detector = Node(
        package='omni_robot',
        executable='target_detector',
        name='target_detector',
        parameters=[{
            'use_sim_time': True,
            'min_blue_area_px': 220,
            'min_visible_blue_pixels': 100,
            'hsv_blue_lower': [112, 170, 70],
            'hsv_blue_upper': [128, 255, 255],
            'blue_channel_min': 120,
            'blue_channel_dominance_min': 60,
            'mask_open_kernel_px': 5,
            'image_timeout_sec': 0.4,
            'scan_index_window': 10,
            'scan_fallback_window': 80,
            'collision_distance_threshold_m': 0.23,
            'collision_min_points': 3,
            'nav_mode_topic': '/target_nav_mode',
        }],
        output='screen',
    )

    target_nav_bridge = Node(
        package='omni_robot',
        executable='target_nav_bridge',
        name='target_nav_bridge',
        parameters=[{
            'use_sim_time': True,
            'goal_pose_topic': '/target_pose',
            'target_visible_topic': '/target_visible',
            'nav_mode_topic': '/target_nav_mode',
            'map_topic': '/map',
            'action_name': '/navigate_to_pose',
            'spin_action_name': '/spin',
            'robot_frame': 'base_link',
            'map_frame': 'map',
            'target_tolerance_m': 1,
            'occupied_threshold': 50,
            'allow_unknown': False,
            'goal_period_sec': 0.25,
            'goal_update_distance_m': 0.10,
            'last_pose_goal_tolerance_m': 0.5,
            'lost_visibility_timeout_sec': 0.8,
            'selected_goal_topic': '/target_nav_goal',
            'selected_goal_marker_topic': '/target_nav_goal_marker',
            'selected_goal_marker_scale_m': 0.2,
        }],
        output='screen',
    )

    # SLAM Toolbox defaults use base_footprint; Gazebo robot only has base_link.
    base_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen',
    )

    return LaunchDescription([
        declare_use_composition,
        base_alias,
        scan_bridge,
        odom_tf_bridge,
        target_detector,
        target_nav_bridge,
        bringup,
        cmd_vel_bridge,
    ])
