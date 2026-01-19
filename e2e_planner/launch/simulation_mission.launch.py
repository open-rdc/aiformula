from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    e2e_planner_dir = get_package_share_directory('e2e_planner')
    
    return LaunchDescription([
        # Run e2e_planner from its own launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(e2e_planner_dir, 'launch', 'e2e_planner.launch.py')
            )
        ),
        
        # Run MPC Node for Simulator
        Node(
            package='path_tracker',
            executable='mpc_node_sim',
            name='mpc_node_sim',
            output='screen',
            parameters=[{
                'max_v': 2.0,
                'max_accel': 1.0, # Slightly gentler acceleration
                'velocity_gain': 1.573,
                'steer_gain': 1.468,
            }]
        )
    ])
