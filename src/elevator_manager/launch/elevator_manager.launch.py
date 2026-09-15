from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='elevator_manager',
            executable='elevator_manager',
            output='screen'
        )
    ])