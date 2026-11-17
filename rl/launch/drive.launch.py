import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context):
    real = LaunchConfiguration('real').perform(context).lower() == 'true'
    model = os.path.expanduser(LaunchConfiguration('model').perform(context))
    drive_args = ['--model', model]
    if real:
        drive_args.append('--real')
    if LaunchConfiguration('stop_on_departure').perform(context).lower() != 'true':
        drive_args.append('--no-stop-on-departure')

    lane_observation = Node(
        package='rl', executable='lane_observation', output='screen',
        parameters=[{'use_sim_time': not real}],
    )
    drive = Node(
        package='rl', executable='drive', output='screen', emulate_tty=True,
        arguments=drive_args, parameters=[{'use_sim_time': not real}],
    )
    return [lane_observation, drive]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', description='走らせるモデル（sac_actor_*.pth か sac_ckpt_*.pth）'),
        DeclareLaunchArgument('real', default_value='false', description='実機で走らせる'),
        DeclareLaunchArgument('stop_on_departure', default_value='true',
                              description='脱線を検出したら止まって Share を待つ（false: 走り続ける）'),
        OpaqueFunction(function=_launch_nodes),
    ])
