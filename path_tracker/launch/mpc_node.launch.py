import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    path_topic_arg = DeclareLaunchArgument(
        'path_topic',
        default_value='e2e_planner/path',
        description='Topic name for the input path'
    )

    mpc_node = Node(
        package='path_tracker',
        executable='mpc_node',
        name='mpc_node',
        output='screen',
        remappings=[
            ('e2e_planner/path', LaunchConfiguration('path_topic'))
        ]
    )

    return LaunchDescription([
        path_topic_arg,
        mpc_node
    ])
