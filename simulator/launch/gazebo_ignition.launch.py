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
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            # RGB camera (color image only)
            '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            # Depth camera (depth image and point cloud only)
            '/depth_image_raw/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/depth_image_raw/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            # Other sensors
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/navsat@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat',
            '/imu_raw@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/cmd_vel_twist@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
        output='screen',
        remappings=[
            ('/depth_image', '/zed/zed_node/depth/depth_registered'),
            ('/depth_image_raw/points', '/zed/zed_node/pointcloud'),
            ('/imu_raw', '/vectornav/imu')
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
        executable='spawner.py',
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
                get_package_share_directory('ros_ign_gazebo'), 'launch'), '/ign_gazebo.launch.py']),
            launch_arguments=[
                ('ign_args', [world_file_path, ' -r'])]
        ),
        steered_to_twist,
        bridge,
        robot_state_publisher,
        TimerAction(
            period=2.0,
            actions=[caster_yaw_position_spawner],
        ),
    ])
