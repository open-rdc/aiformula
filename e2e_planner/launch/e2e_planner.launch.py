from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='e2e_planner',
            executable='inference_node',
            name='inference_node',
            output='screen',
            parameters=[{
                'model_name': 'model.pt',
                'interval_ms': 100,
            }]
        )
    ])
