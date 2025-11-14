import os
import subprocess
import yaml
import launch
from launch import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # パラメータファイルのパス設定
    config_file_path = os.path.join(
        get_package_share_directory('main_executor'),
        'config',
        'main_params.yaml'
    )

    # 起動パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        launch_params = yaml.safe_load(file)['launch']['ros__parameters']

    # メイン実行機ノードの作成
    main_exec_node = Node(
        package = 'main_executor',
        executable = 'main_exec',
        parameters = [config_file_path],
        output='screen'
    )
    # socketcanノードの作成
    socketcan_node = Node(
        package = 'socketcan_interface',
        executable = 'socketcan_interface_node',
        parameters = [config_file_path],
        output='screen'
    )
    # 操縦機ノードの作成
    joy_node = Node(
        package = 'joy',
        executable = 'joy_node',
        output='screen'
    )

    # vectornav起動の作成
    vectornav_launch = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource([os.path.join(
            get_package_share_directory('vectornav'), 'launch/'),
            'vectornav.launch.py'])
    )
    # odrive起動の作成
    odrive_launch = launch.actions.IncludeLaunchDescription(
        AnyLaunchDescriptionSource([os.path.join(
            get_package_share_directory('odrive_can'), 'launch/'),
            'own_launch.yaml'])
    )

    # 起動エンティティクラスの作成
    launch_discription = LaunchDescription()

    # 起動の追加
    if(launch_params['joy'] is True):
        launch_discription.add_action(joy_node)
    if(launch_params['vectornav'] is True):
        launch_discription.add_action(vectornav_launch)
    if(launch_params['odrive'] is True):
        launch_discription.add_action(odrive_launch)
    if(launch_params['socketcan'] is True):
        launch_discription.add_action(socketcan_node)

    launch_discription.add_action(main_exec_node)

    return launch_discription
