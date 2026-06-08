from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    inference_node = Node(
        package='e2e_planner',
        executable='inference_node',
        name='inference_node',
        output='screen',
        parameters=[{
            'model_name': 'e2e_model.pt',
            'interval_ms': 100,
        }]
    )

    return LaunchDescription([inference_node])
