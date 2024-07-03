import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='line_follower_package',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='line_follower_package',
            executable='line_follower_node',
            name='line_follower_node',
            output='screen'
        ),
        Node(
            package='line_follower_package',
            executable='motor_control_node',
            name='motor_control_node',
            output='screen'
        ),
    ])
