from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="wall_following_pkg",
            executable="wall_follower_node",
            output="screen",
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])