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
        bringup,
        cmd_vel_bridge,
    ])
