from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Science
        Node(
            package='science',
            executable='science',
        )
    ])
