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

    mpc_node_sim = Node(
        package='path_tracker',
        executable='mpc_node_sim',
        name='mpc_node_sim',
        output='screen',
        remappings=[
            ('e2e_planner/path', LaunchConfiguration('path_topic'))
        ],
        parameters=[{
            'velocity_gain': 1.0,
            'steer_gain': 1.0,
            'max_v': 4.0,
            'max_accel': 2.0,
            'goal_tolerance': 0.1
        }]
    )

    return LaunchDescription([
        path_topic_arg,
        mpc_node_sim
    ])
