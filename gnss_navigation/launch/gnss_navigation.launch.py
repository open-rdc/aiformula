# import statements
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    gnss_nav_dir = get_package_share_directory("gnss_navigation")
    config_dir = os.path.join(gnss_nav_dir, 'config')
    course_dir = os.path.join(config_dir, "course_data")
    # course_data = os.path.join(course_dir, "0425_test_curve.csv")
    course_data = os.path.join(course_dir, "0425_test_straight.csv")

    print("use course_data: " + course_data + "\n")

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
                {'file_path': course_data}
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
            name='static_tf_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            condition=IfCondition(LaunchConfiguration('publish_transform'))
        )
    ])

