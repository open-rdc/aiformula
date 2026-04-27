"""
データ収集用ランチファイル。実機・シミュレーター両方に対応。

使い方:
  # シミュレーター
  ros2 launch e2e_planner data_collector.launch.py sim:=true

  # 実機 (ZED カメラ ROS トピック)
  ros2 launch e2e_planner data_collector.launch.py sim:=false

操作:
  buttons[0]: 録画開始/停止トグル
  buttons[1]: 右折ラベル (押している間)
  buttons[2]: 左折ラベル (押している間)
  ボタン未押下: 道なりラベル

パラメータ:
  sim:=true/false   シミュレーター or 実機
  save_dir          保存先ディレクトリ
  image_topic       カメラトピック (sim時: /image_raw, 実機: /zed/zed_node/rgb/image_rect_color)
  pose_topic        姿勢トピック   (sim時: /odom, 実機: /zed/zed_node/odom)
  joy_topic         ジョイスティックトピック (default: /joy)
  save_interval_sec 保存間隔[秒] (default: 0.1 = 10 Hz)
  sdk_flag          ZED SDK 直接使用 [実機のみ有効] (default: false)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def _launch_setup(context, *args, **kwargs):
    sim_flag = LaunchConfiguration('sim').perform(context).lower() == 'true'
    save_dir = LaunchConfiguration('save_dir').perform(context)

    # トピック既定値をシム/実機で切り替え
    image_topic = LaunchConfiguration('image_topic').perform(context)
    pose_topic  = LaunchConfiguration('pose_topic').perform(context)
    joy_topic   = LaunchConfiguration('joy_topic').perform(context)
    save_interval = float(LaunchConfiguration('save_interval_sec').perform(context))
    sdk_flag = LaunchConfiguration('sdk_flag').perform(context).lower() == 'true'

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    if sim_flag:
        # シミュレーター用
        if not image_topic:
            image_topic = '/image_raw'
        if not pose_topic:
            pose_topic = '/odom'

        collector_node = Node(
            package='e2e_planner',
            executable='data_collector_sim',
            name='data_collector_sim',
            output='screen',
            parameters=[{
                'save_dir':          save_dir,
                'image_topic':       image_topic,
                'odom_topic':        pose_topic,
                'joy_topic':         joy_topic,
                'save_interval_sec': save_interval,
            }],
        )
    else:
        # 実機用
        if not image_topic:
            image_topic = '/zed/zed_node/rgb/image_rect_color'
        if not pose_topic:
            pose_topic = '/zed/zed_node/odom'

        collector_node = Node(
            package='e2e_planner',
            executable='data_collector',
            name='data_collector',
            output='screen',
            parameters=[{
                'save_dir':          save_dir,
                'image_topic':       image_topic,
                'pose_topic':        pose_topic,
                'joy_topic':         joy_topic,
                'save_interval_sec': save_interval,
                'sdk_flag':          sdk_flag,
            }],
        )

    return [joy_node, collector_node]


def generate_launch_description() -> LaunchDescription:
    home = str(Path.home())
    return LaunchDescription([
        DeclareLaunchArgument('sim',              default_value='false',
                              description='true=シミュレーター, false=実機'),
        DeclareLaunchArgument('save_dir',         default_value=f'{home}/ros2_ws/dataset',
                              description='データ保存先ディレクトリ'),
        DeclareLaunchArgument('image_topic',      default_value='',
                              description='カメラトピック (空文字=sim/実機で自動選択)'),
        DeclareLaunchArgument('pose_topic',       default_value='',
                              description='姿勢トピック   (空文字=sim/実機で自動選択)'),
        DeclareLaunchArgument('joy_topic',        default_value='/joy'),
        DeclareLaunchArgument('save_interval_sec', default_value='0.1'),
        DeclareLaunchArgument('sdk_flag',         default_value='false',
                              description='ZED SDK 直接使用 [実機のみ有効]'),
        OpaqueFunction(function=_launch_setup),
    ])
