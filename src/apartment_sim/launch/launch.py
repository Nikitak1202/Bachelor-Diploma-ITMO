from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apartment_sim',
            executable='obstacles_controller',
            name='obstacles_controller',
            output='screen',
            parameters=[]
        )
    ])