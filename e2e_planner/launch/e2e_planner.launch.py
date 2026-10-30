import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def resolve_python() -> str:
    """ROS の python (/usr/bin/python3) には torch が入っていないため、
    torch のある python があればそれを prefix にして起動する。
    見つからなければ空文字を返し、従来どおり shebang の python に任せる。"""
    candidates = [
        os.environ.get('E2E_PLANNER_PYTHON', ''),
        os.path.expanduser('~/miniforge3/envs/aiformula/bin/python'),
    ]
    for candidate in candidates:
        if candidate and os.path.exists(candidate):
            return candidate
    return ''


def launch_setup(context, *args, **kwargs):
    sim_flag = LaunchConfiguration('sim_flag').perform(context)

    if sim_flag.lower() == 'true':
        executable_name = 'inference_node_sim.py'
    else:
        executable_name = 'inference_node'

    inference_node = Node(
        package='e2e_planner',
        executable=executable_name,
        name='inference_node',
        output='screen',
        prefix=resolve_python(),
        parameters=[{
            'model_name': LaunchConfiguration('model_name').perform(context),
            'num_mask_channels': int(LaunchConfiguration('num_mask_channels').perform(context)),
            'device': LaunchConfiguration('device').perform(context),
            'interval_ms': 100,
        }]
    )

    return [inference_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_name',
            default_value='e2e_model_pylon.pt',
            description='weights/ 以下の重みファイル名'
        ),
        DeclareLaunchArgument(
            'num_mask_channels',
            default_value='2',
            description='重みの入力のマスク数。2 = 白線 + パイロン、1 = 白線だけ (feat/rgb_mask の重み)'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cuda',
            description='推論に使うデバイス。GPU の無い PC で確かめるときだけ cpu'
        ),
        DeclareLaunchArgument(
            'sim_flag',
            default_value='false',
            description='Flag to use simulation inference node'
        ),
        OpaqueFunction(function=launch_setup)
    ])
