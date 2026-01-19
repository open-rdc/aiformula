from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='e2e_planner',
            executable='image_binarizer_node',
            name='image_binarizer_node',
            output='screen',
            parameters=[{
                'threshold_value': 240,
                'max_value': 255,
            }]
        ),
        Node(
            package='e2e_planner',
            executable='inference_node',
            name='inference_node',
            output='screen',
            parameters=[{
                'model_name': 'e2e_model (2).pt',
                'interval_ms': 100,
                'debug_mode': True,
                'sdk_flag': False,
            }],
            remappings=[
                ('/image_raw', '/zed/zed_node/rgb/image_rect_color'),
            ]
        )
    ])
