from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apartment_sim',
            executable='obstacles_controller',
            name='obstacles_controller',
            output='screen',
            parameters=[{
                'number_of_obstacles': 3,
                'target_speed': 0.5,
                'obstacle_speed': 0.3,
                'update_rate': 2.0,
                'target_spawn_x': 3.2,
                'target_spawn_y': 2.0,
                'obstacle_spawn_positions': [
                    0.8, 0.8,
                    0.8, 3.2,
                    -0.8, 2.0,
                ],
            }]
        )
    ])