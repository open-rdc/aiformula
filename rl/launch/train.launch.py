import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context):
    real = LaunchConfiguration('real').perform(context).lower() == 'true'
    topomap = LaunchConfiguration('topomap').perform(context).lower() == 'true'
    model_dir = os.path.expanduser(LaunchConfiguration('model_dir').perform(context))

    train_args = ['--model-dir', model_dir,
                  '--episode-sec', LaunchConfiguration('episode_sec').perform(context)]
    if real:
        train_args.append('--real')
    if topomap:
        train_args.append('--topomap')
    save_name = LaunchConfiguration('save_name').perform(context)
    if save_name:
        train_args += ['--save-name', save_name]
    resume = LaunchConfiguration('resume').perform(context)
    if resume:
        train_args += ['--resume', os.path.expanduser(resume)]
    if LaunchConfiguration('auto_reset').perform(context).lower() == 'true':
        train_args.append('--auto-reset')

    lane_observation = Node(
        package='rl',
        executable='lane_observation',
        output='screen',
        parameters=[{'use_sim_time': not real}],
    )
    train = Node(
        package='rl',
        executable='train',
        output='screen',
        emulate_tty=True,
        arguments=train_args,
        parameters=[{'use_sim_time': not real}],
    )
    return [lane_observation, train]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('real', default_value='false', description='実機で学習する'),
        DeclareLaunchArgument('topomap', default_value='false', description='報酬にトポロジカルマップを使う'),
        DeclareLaunchArgument('auto_reset', default_value='false',
                              description='シムで車を自動で初期位置に戻す（false: Share で手動にして人が戻し、Share で自動に戻したら再開）'),
        DeclareLaunchArgument('episode_sec', default_value='300.0',
                              description='エピソードの制限時間 [s]（自動モードで走り始めてから）'),
        DeclareLaunchArgument('save_name', default_value='',
                              description='Ctrl-C で保存するファイル名（sac_actor_<名前>.pth / sac_ckpt_<名前>.pth）。空なら日時_ep番号'),
        DeclareLaunchArgument('resume', default_value='',
                              description='再開するモデル（sac_ckpt_*.pth か sac_actor_*.pth）。空なら最初から'),
        DeclareLaunchArgument('model_dir', default_value='~/formula_ws/src/rl/rl/train/models',
                              description='モデルの保存先'),
        OpaqueFunction(function=_launch_nodes),
    ])
