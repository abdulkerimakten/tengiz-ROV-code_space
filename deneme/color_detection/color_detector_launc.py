from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='color_detection',
            executable='color_detector',
            name='color_detector'
        ),
        Node(
            package='color_detection',
            executable='color_publisher',
            name='color_publisher'
        )
    ])


