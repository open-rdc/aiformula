import os
import subprocess
import yaml
import launch
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    # CANインタフェース起動ファイルのパス設定
    can_launch_path = os.path.join(
        get_package_share_directory('socketcan_interface'),
        'config',
        'can_up.sh'
    )

    # 起動パラメータファイルのロード
    with open(config_file_path, 'r') as file:
        launch_params = yaml.safe_load(file)['launch']['ros__parameters']


    # メイン実行機のログレベルの設定
    log_level = LaunchConfiguration("log_level")
    # デフォルトのログレベルを'info'に設定
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )
    # メイン実行機のシミュレータ環境の設定
    sim_flag = LaunchConfiguration("sim_flag")
    # デフォルトのシミュレータ環境を'false'に設定
    sim_flag_arg = DeclareLaunchArgument(
        "sim_flag",
        default_value=["false"],
        description="Use simulator or not",
    )

    # メイン実行機ノードの作成
    main_exec_node = Node(
        package = 'main_executor',
        executable = 'main_exec',
        parameters = [config_file_path],
        arguments=["--sim-flag", sim_flag, "--ros-args", "--log-level", log_level],
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
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('vectornav'), 'launch/'),
            'vectornav.launch.py'])
    )

    # # zed-ros2-wrapper起動の作成
    # zed_wrapper_launch = launch.actions.IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('zed-ros2-wraper'), 'launch'),
    #         'zed_camera.launch.py'])
    # )

    # 起動エンティティクラスの作成
    launch_discription = LaunchDescription()

    # 起動の追加
    if(launch_params['can'] is True):
        subprocess.run(['sudo', 'sh', can_launch_path])
    if(launch_params['joy'] is True):
        launch_discription.add_entity(joy_node)
    if(launch_params['vectornav'] is True):
        launch_discription.add_action(vectornav_launch)
    if(launch_params['zed'] is True):
        launch_discription.add_action(zed_wrapper_launch)

    launch_discription.add_action(log_level_arg)
    launch_discription.add_action(sim_flag_arg)
    launch_discription.add_entity(main_exec_node)

    return launch_discription
