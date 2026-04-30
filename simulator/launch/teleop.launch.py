import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('simulator'),
        'config',
        'teleop_params.yaml',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        remappings=[('cmd_vel', '/cmd_vel_twist')],
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([joy_node, teleop_node])
