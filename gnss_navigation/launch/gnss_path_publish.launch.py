# import statements
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
          'publish_transform',
          default_value='true',
          description='Whether to publish static transform from odom to base_link'
        ),
        Node(
            package='gnss_navigation',
            executable='gnss_path_publisher',
            name='gnss_path_publisher',
            parameters=[
                {'file_path': '/home/ros2_ws/src/AIFormula_private/gnss_navigation/config/course_data/gazebo_shihou_course.csv'}
            ]
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
            name='static_tf_pub_odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            condition=IfCondition(LaunchConfiguration('publish_transform'))
        )
    ])
