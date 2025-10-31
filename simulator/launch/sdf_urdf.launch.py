from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    world_file_path = PathJoinSubstitution([
        get_package_share_directory('simulator'),
        'world',
        'shihou_world.sdf'
    ])

    # === URDFファイルパス ===
    urdf_file_path = os.path.join(
        get_package_share_directory('simulator'),
        'urdf',
        'motor_spin.urdf'
    )

    # === robot_state_publisher ===
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(urdf_file_path).read()
        }]
    )

    # --- controller spawner ---
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    motor_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['motor_effort_controller'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/depth_image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/depth_points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/navsat@sensor_msgs/msg/NavSatFix@ignition.msgs.NavSat',
            '/imu_raw@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
        output='screen'
    )

    # ✅ 元の構造は一切変更せずにURDF関連ノードをreturn内に追加
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                ('gz_args', [world_file_path, ' -r'])]
        ),
        robot_state_publisher,  # ← ここだけ追加
        joint_state_spawner,
        motor_controller_spawner,        # ← ここだけ追加
        bridge
    ])

