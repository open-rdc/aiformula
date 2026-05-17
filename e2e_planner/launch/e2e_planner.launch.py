from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    sim_flag = LaunchConfiguration('sim_flag').perform(context)
    normalization_dataset_path = LaunchConfiguration('normalization_dataset_path').perform(context)

    is_sim = sim_flag.lower() == 'true'
    executable_name = 'inference_node'
    model_name = 'e2e_model.pt'

    inference_node = Node(
        package='e2e_planner',
        executable=executable_name,
        name='inference_node',
        output='screen',
        parameters=[{
            'model_name': model_name,
            'interval_ms': 50,
            'sim_flag': is_sim,
            'image_topic': '/image_raw',
            'debug_mode': True,
            'default_command': 1,
            'use_place_recognition': True,
            'yolop_input_size': 256,
            'yolop_fp16': True,
            'placenet_model_name': 'placenet.pt',
            'topomap_dir_name': 'topomap',
            'placenet_delta': 10.0,
            'placenet_window_lower': -1,
            'placenet_window_upper': 10,
            'normalization_dataset_path': normalization_dataset_path,
        }]
    )

    return [inference_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_flag',
            default_value='false',
            description='Flag to use simulation inference node'
        ),
        DeclareLaunchArgument(
            'normalization_dataset_path',
            default_value='',
            description='Dataset path used to compute waypoint normalization bounds'
        ),
        OpaqueFunction(function=launch_setup)
    ])
