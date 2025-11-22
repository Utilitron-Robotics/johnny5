from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_type',
            default_value='aloha',
            description='Type of robot to launch: "aloha" or "johnny5"'
        ),
        DeclareLaunchArgument(
            'left_port',
            default_value='/dev/am_arm_follower_left',
            description='Port for the Left Arm bus'
        ),
        DeclareLaunchArgument(
            'right_port',
            default_value='/dev/am_arm_follower_right',
            description='Port for the Right Arm bus'
        ),
        Node(
            package='alohamini_ros2',
            executable='aloha_node',
            name='aloha_node',
            output='screen',
            parameters=[{
                'robot_type': LaunchConfiguration('robot_type'),
                'left_port': LaunchConfiguration('left_port'),
                'right_port': LaunchConfiguration('right_port')
            }]
        )
    ])