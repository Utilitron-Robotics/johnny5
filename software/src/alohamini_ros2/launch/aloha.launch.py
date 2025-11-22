from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='alohamini_ros2',
            executable='aloha_node',
            name='aloha_node',
            output='screen',
            parameters=[
                {'left_port': '/dev/am_arm_follower_left'},
                {'right_port': '/dev/am_arm_follower_right'}
            ]
        )
    ])