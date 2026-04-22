import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_omni_robot = FindPackageShare('omni_robot').find('omni_robot')
    xacro_file = os.path.join(pkg_omni_robot, 'urdf', 'omni_robot.urdf.xacro')

    # Single source of truth for default spawn pose.
    default_spawn_x = '2.0'
    default_spawn_y = '0.0'
    default_spawn_z = '0.1'
    default_spawn_yaw = '0.0'
    
    # Generate URDF file in /tmp (writeable inside container)
    urdf_file = '/tmp/omni_robot.urdf'
    generate_urdf = ExecuteProcess(
        cmd=['xacro', xacro_file, '-o', urdf_file],
        output='screen'
    )

    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    # Robot state publisher using the generated URDF file
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_file],
        output='screen'
    )

    # Spawn entity in Gazebo using the generated URDF file
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'omni_robot',
            '-file', urdf_file,
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
            '-robot_namespace', 'omni_robot'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('spawn_x', default_value=default_spawn_x),
        DeclareLaunchArgument('spawn_y', default_value=default_spawn_y),
        DeclareLaunchArgument('spawn_z', default_value=default_spawn_z),
        DeclareLaunchArgument('spawn_yaw', default_value=default_spawn_yaw),
        generate_urdf,
        rsp_node,
        spawn_entity
    ])