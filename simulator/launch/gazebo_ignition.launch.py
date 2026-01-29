from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

import os


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
            ('/odom', '/zed/zed_node/odom'),
            ('/depth_image', '/zed/zed_node/depth/depth_registered'),
            ('/depth_image_raw/points', '/zed/zed_node/pointcloud'),
        ]
    )

    ackermann_to_twist = Node(
        package='simulator',
        executable='ackermann_to_twist.py',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/cmd_vel_twist',
            'wheel_base': 0.8,
        }]
    )

    return LaunchDescription([
        world_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                ('gz_args', [world_file_path, ' -r'])]
        ),
        ackermann_to_twist,
        bridge
    ])
