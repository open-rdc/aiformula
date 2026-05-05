from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

import os
import shutil


def generate_launch_description():
    # Declare world argument (default: shihou_world.sdf)
    # Available options: shihou_world.sdf, classic_world_ignition.sdf
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='shihou_world.sdf',
        description='World file name (e.g., shihou_world.sdf, classic_world_ignition.sdf)'
    )

    world_file_path = PathJoinSubstitution([
        get_package_share_directory('simulator'),
        'world',
        LaunchConfiguration('world')
    ])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # RGB camera (color image only)
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            # Depth camera (depth image and point cloud only)
            '/depth_image_raw/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_image_raw/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            # Other sensors
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            '/imu_raw@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/cmd_vel_twist@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen',
        remappings=[
            ('/image_raw', '/zed/zed_node/rgb/image_rect_color'),
            ('/camera_info', '/zed/zed_node/rgb/camera_info'),
            ('/depth_image', '/zed/zed_node/depth/depth_registered'),
            ('/navsat', '/vectornav/gnss'),
            ('/depth_image_raw/points', '/zed/zed_node/point_cloud'),
            # /imu_raw は convert_sim_to_vectornav_pose.py が yaw オフセットを適用して
            # /vectornav/imu へ再配信するため、bridge ではリマップしない
        ]
    )

    steered_to_twist = Node(
        package='simulator',
        executable='steered_to_twist.py',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/cmd_vel_twist',
            'wheel_base': 0.8,
        }]
    )

    convert_vectornav_pose = Node(
        package='simulator',
        executable='convert_sim_to_vectornav_pose.py',
        output='screen',
        parameters=[{
            # /imu_raw から /vectornav/imu へ変換するときのyaw補正。
            # /vectornav/imuのyawは実機同様に北基準headingとして扱い，
            # localization/odom側でROS ENU yawへ変換する。
            'imu_frame_id': 'base_link',
        }]
    )

    convert_vectornav_velocity_body = Node(
        package='simulator',
        executable='convert_sim_to_vectornav_velocity_body.py',
        output='screen',
        parameters=[{
            'body_frame_id': 'base_link',
        }]
    )

    ros2_control_src = os.path.join(
        get_package_share_directory('simulator'),
        'models',
        'ai_car1',
        'ros2_control.yaml',
    )
    ros2_control_dst = '/tmp/simulator_ai_car1_ros2_control.yaml'
    shutil.copyfile(ros2_control_src, ros2_control_dst)

    caster_yaw_position_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'caster_yaw_position_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '60',
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                ('gz_args', [world_file_path, ' -r'])]
        ),
        steered_to_twist,
        convert_vectornav_pose,
        convert_vectornav_velocity_body,
        bridge,
        TimerAction(
            period=2.0,
            actions=[caster_yaw_position_spawner],
        ),
    ])
