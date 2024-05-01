# import statements
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gnss_navigation',
            executable='gnss_path_publisher',
            name='gnss_path_publisher',
            parameters=[
                {'file_path': '/home/ros2_ws/src/AIFormula_private/gnss_navigation/config/course_data/gazebo_shihou_course.csv'}
            ]
        )
    ])
