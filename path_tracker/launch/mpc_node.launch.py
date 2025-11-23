import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    # We can add arguments for parameters here if needed

    mpc_node = Node(
        package='path_tracker',
        executable='mpc_node',
        name='mpc_node',
        output='screen'
    )

    return LaunchDescription([
        mpc_node
    ])
