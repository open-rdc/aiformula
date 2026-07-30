from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

import os
import shutil


def generate_launch_description():
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
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_image_raw/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_image_raw/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            '/imu_raw@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/cmd_vel_twist@geometry_msgs/msg/Twist@gz.msgs.Twist', 
            '/obstacle/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            '/obstacle/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/gnss_path@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V',
            '/origin_gnss_path@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V'],
        output='screen',
        remappings=[
            ('/image_raw', '/zed/zed_node/rgb/image_rect_color'),
            ('/depth_image', '/zed/zed_node/depth/depth_registered'),
            ('/depth_image_raw/points', '/zed/zed_node/point_cloud'),
        ]
    )

    # gz-simの点群プラグインはignition_frame_idを無視し、常に"model/link/sensor"の
    # スコープ付き名前をheader.frame_idに書き込む（実機zed_wrapperはframe_id="camera_depth_link"
    # を使うため、シム環境ではTFに存在しない名前になりlookupTransformが失敗する）。
    # robot_state_publisherが持つ実TFツリー上のcamera_depth_linkへ恒等変換でブリッジする。
    camera_depth_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id', 'camera_depth_link',
            '--child-frame-id', 'ai_car1/camera_depth_link/depth_camera',
        ],
        output='screen',
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
            'imu_frame_id': 'vectornav',
        }]
    )

    convert_vectornav_velocity_body = Node(
        package='simulator',
        executable='convert_sim_to_vectornav_velocity_body.py',
        output='screen',
        parameters=[{
            'frame_id': 'vectornav',
        }]
    )

    urdf_path = os.path.join(
        get_package_share_directory('simulator'),
        'models',
        'ai_car1',
        'model.urdf',
    )
    ros2_control_src = os.path.join(
        get_package_share_directory('simulator'),
        'models',
        'ai_car1',
        'ros2_control.yaml',
    )
    ros2_control_dst = '/tmp/simulator_ai_car1_ros2_control.yaml'
    shutil.copyfile(ros2_control_src, ros2_control_dst)
    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    caster_yaw_position_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'caster_yaw_position_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '60',
            '--switch-timeout',
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
        bridge,
        camera_depth_frame_bridge,
        robot_state_publisher,
        convert_vectornav_pose,
        convert_vectornav_velocity_body,
        TimerAction(
            period=2.0,
            actions=[caster_yaw_position_spawner],
        ),
    ])
