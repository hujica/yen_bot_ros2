from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yen_bot',
            executable='gui_node',
            name='yen_bot_gui',
            output='screen',
        ),
        Node(
            package='yen_bot',
            executable='hardware_node',
            name='yen_bot_hardware',
            output='screen',
        ),
        Node(
            package='yen_bot',
            executable='ros_node',
            name='yen_bot_ros',
            output='screen',
        ),
        Node(
            package='yen_bot',
            executable='kinematics_node',
            name='yen_bot_kinematics',
            output='screen',
        ),
    ])
