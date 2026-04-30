"""
トポロジカルマップを使ったシミュレーターナビゲーション用ランチファイル。

使い方:
  ros2 launch e2e_planner topo_nav.launch.py \
    topo_map:=/path/to/session/topo_map.json

オプション:
  model_name:=junction_classifier.pt  分類器モデル (weights/ 以下)
  speed:=1.0                          走行速度 [m/s]
  lookahead_dist:=1.5                 先読み距離 [m]
  use_yolop:=false                    YOLOP使用 (false=シム用赤色抽出)
  loop_route:=false                   完走後に先頭に戻る
  wheel_base:=0.8                     ホイールベース [m]
  max_steer:=0.5                      最大舵角 [rad]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            'topo_map',
            description='topo_map.json の絶対パス (必須)',
        ),
        DeclareLaunchArgument(
            'model_name',
            default_value='junction_classifier.pt',
            description='weights/ 以下の分類器モデルファイル名 (空文字=スキップ)',
        ),
        DeclareLaunchArgument('speed',          default_value='1.0'),
        DeclareLaunchArgument('lookahead_dist', default_value='1.5'),
        DeclareLaunchArgument('use_yolop',      default_value='true'),
        DeclareLaunchArgument('loop_route',     default_value='false'),
        DeclareLaunchArgument('wheel_base',     default_value='0.8'),
        DeclareLaunchArgument('max_steer',      default_value='0.8'),
        DeclareLaunchArgument('interval_ms',    default_value='100'),
        DeclareLaunchArgument('odom_topic',     default_value='/odom'),
        DeclareLaunchArgument('image_topic',    default_value='/image_raw'),
    ]

    topo_nav_node = Node(
        package='e2e_planner',
        executable='topo_nav_node_sim',
        name='topo_nav_node_sim',
        output='screen',
        parameters=[{
            'topo_map':       LaunchConfiguration('topo_map'),
            'model_name':     LaunchConfiguration('model_name'),
            'speed':          LaunchConfiguration('speed'),
            'lookahead_dist': LaunchConfiguration('lookahead_dist'),
            'use_yolop':      LaunchConfiguration('use_yolop'),
            'loop_route':     LaunchConfiguration('loop_route'),
            'wheel_base':     LaunchConfiguration('wheel_base'),
            'max_steer':      LaunchConfiguration('max_steer'),
            'interval_ms':    LaunchConfiguration('interval_ms'),
            'odom_topic':     LaunchConfiguration('odom_topic'),
            'image_topic':    LaunchConfiguration('image_topic'),
        }],
    )

    return LaunchDescription(args + [topo_nav_node])
