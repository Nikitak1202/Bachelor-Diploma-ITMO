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
                'number_of_obstacles': 5,
                'target_speed': 0.5,
                'obstacle_speed': 0.3,
                'update_rate': 2.0,
                'spawn_area_min_x': -5.0,
                'spawn_area_max_x': 5.0,
                'spawn_area_min_y': -5.0,
                'spawn_area_max_y': 5.0,
            }]
        )
    ])