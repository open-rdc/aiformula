# file: simulator/launch/rviz_aerial.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg = get_package_share_directory('simulator')

    # ---- Launch args ----
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg, 'rviz', 'odom.rviz']),  # 任意: 用意していれば .rviz のフルパスを渡せる
        description='(optional) RViz config file path'
    )
    urdf_arg = DeclareLaunchArgument(
        'urdf',
        default_value=PathJoinSubstitution([pkg, 'urdf', 'aerial_bg.urdf']),
        description='URDF path for aerial background'
    )

    rviz_config = LaunchConfiguration('rviz_config')
    urdf = LaunchConfiguration('urdf')

    # ---- robot_state_publisher (URDFを配信) ----
    # URDFファイルを読み込んで robot_description に流す
    urdf_path_default = os.path.join(pkg, 'urdf', 'aerial_bg.urdf')
    with open(urdf_path_default, 'r') as f:
        urdf_text_default = f.read()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='aerial_bg_state_pub',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # urdf 引数を使った動的読込を簡単にするため、デフォルト内容をそのまま入れておく
            # （引数で別URDFを渡したい場合は、このノードをxacro等に差し替えてもOK）
            'robot_description': urdf_text_default
        }]
    )

    # ---- static TF: map -> ai_car1/odom ----
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_ai_car1_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'ai_car1/odom'],
        output='screen'
    )

    # ---- RViz ----
    # rviz_config が与えられたら -d で使う。未指定なら素のRVizを起動。
    rviz_args = ['-d', rviz_config]  # 空文字でも問題なく起動します

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=rviz_args,
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        rviz_config_arg,
        urdf_arg,
        rsp,
        static_tf,
        rviz,
    ])

