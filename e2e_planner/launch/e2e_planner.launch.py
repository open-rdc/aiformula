from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    sim_flag = LaunchConfiguration('sim_flag').perform(context)

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
            # 学習時 (scripts/binarize_dataset.py) は YOLOPv2Processor を引数省略で生成しており
            # input_size=640 / use_fp16=False。letterbox の縮小率は 640 のときだけ 1.0 になり、
            # 640x360 の入力が無縮小で YOLOP に入る。256 だと 256x144 まで潰れてマスクが変わる。
            'yolop_input_size': 640,
            'yolop_fp16': False,
            'placenet_model_name': 'placenet.pt',
            'topomap_dir_name': 'topomap',
            'placenet_delta': 5.0,
            'placenet_window_lower': -1,
            'placenet_window_upper': 10,
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
        OpaqueFunction(function=launch_setup)
    ])
