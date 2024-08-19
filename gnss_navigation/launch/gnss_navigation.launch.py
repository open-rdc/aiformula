# import statements
import os
import subprocess
import yaml
import datetime

import launch
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # gnss_navigationのパス設定
    gnss_nav_dir = get_package_share_directory('gnss_navigation')
    config_dir = os.path.join(gnss_nav_dir, 'config')
    rviz_config_dir = os.path.join(config_dir, 'rviz', 'gnss_navigation.rviz')
    vectornav_launch_path = os.path.join(gnss_nav_dir, 'config', 'bringup_vectornav.sh')

    # 起動パラメータファイルのロード
    params_file_path = os.path.join(config_dir, 'params', 'gnss_nav_params.yaml')
    with open(params_file_path, 'r') as file:
        gnss_nav_params = yaml.safe_load(file)['launch']['ros__parameters']

    # odometry_publisherのパス設定
    odom_pub_pkg_dir = get_package_share_directory('odometry_publisher')
    odom_pub_launch_dir = os.path.join(odom_pub_pkg_dir, 'launch')

    # vectornavのパス設定
    vectornav_pkg_dir = get_package_share_directory('vectornav')
    vectornav_launch_dir = os.path.join(vectornav_pkg_dir, 'launch')

    # バグファイルの設定
    now = datetime.datetime.now()
    formatted_datetime = now.strftime('%Y_%m_%d-%H_%M_%S')
    bag_file = os.path.expanduser('~/rosbag/rosbag2_' + formatted_datetime)

    # コースデータの設定
    course_data = os.path.join(config_dir, 'course_data', 'gazebo_shihou_course.csv')
    # course_data = os.path.join(config_dir, 'course_data', '0425_test_straight.csv')

    print("-"*50)
    print("use course_data: " + course_data)
    print("bag file path: " + bag_file)
    print("-"*50)

    # パラメータ参照
    use_publish_tf = LaunchConfiguration('use_publish_tf')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rosbag = LaunchConfiguration('use_rosbag')

    # パラメータ定義
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value=str(gnss_nav_params['use_sim_time']),
        description='Use simulation (Gazebo) clock if true')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value=str(gnss_nav_params['use_rviz']),
        description='Use rviz')
    declare_use_rosbag = DeclareLaunchArgument(
        'use_rosbag',
        default_value=str(gnss_nav_params['use_rosbag']),
        description='Recording rosbag')
    declare_publish_tf = DeclareLaunchArgument(
        'use_publish_tf',
        default_value=str(gnss_nav_params['use_publish_tf']),
        description='Whether to publish static transform from map to odom')

    # 他のパッケージのlaunch立ち上げ
    other_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([vectornav_launch_dir, '/vectornav.launch.py'])
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([odom_pub_launch_dir, '/gyro_odometry.launch.py']),
                launch_arguments={
                    'use_rviz': 'false',
                    'use_rosbag': 'false',
                    'sub_can_frame_topic_nanme': '/can_rx_711',
                    'sub_imu_topic_name': '/vectornav/imu',
                    'pub_odometry_topic_name': '/aiformula_sensing/gyro_odometry/odom',
                    'odom_frame_id': 'odom',
                    'robot_frame_id': 'base_footprint'
                }.items(),
            )
        ]
    )

    # ノードの作成
    launch_node = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
            package='gnss_navigation',
            executable='gnss_path_publisher',
            name='gnss_path_publisher',
            parameters=[{'file_path': course_data}]
            ),
            Node(
                package='gnss_navigation',
                executable='gnss_path_follower',
                name='gnss_path_follower',
                output='screen'
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_map_to_odom',
                arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
                condition=IfCondition(use_publish_tf)
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_dir],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
                condition=IfCondition(use_rviz)
            )
        ]
    )

    # vectornavへ実行権限の付与
    # subprocess.run(['sudo', 'sh', vectornav_launch_path])

    # 起動エンティティ作成，起動項目の追加
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rosbag)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_publish_tf)

    # ld.add_action(other_launch)

    ld.add_action(launch_node)

    # rosbag開始
    # ld.add_action(launch.actions.ExecuteProcess(
    #     cmd=['ros2', 'bag', 'record', '-a', '-o', bag_file],
    #     output='screen',
    #     condition=IfCondition(use_rosbag)
    # ))

    return ld
