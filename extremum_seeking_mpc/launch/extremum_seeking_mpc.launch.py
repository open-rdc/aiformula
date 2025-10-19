import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    PACKAGE_NAME = "extremum_seeking_mpc"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)

    # Define frame IDs and topic names directly
    FRAME_IDS = {
        "base_footprint": "base_footprint"
    }

    TOPIC_NAMES = {
        "perception": {
            "lane_lines": {
                "left": "/lane_points/left",
                "right": "/lane_points/right"
            },
            "objects": {
                "info": "/obstacle_detector/obstacle_info"
            }
        },
        "sensing": {
            "odometry": {
                "gyro": "/odom"
            }
        }
    }

    ROS_PARAM_CONFIG = (
        osp.join(PACKAGE_DIR, "config", "extremum_seeking_mpc_params.yaml"),
        osp.join(PACKAGE_DIR, "config", "extremum_seeking_controller_params.yaml"),
        osp.join(PACKAGE_DIR, "config", "object_risk_calculator_params.yaml"),
        osp.join(PACKAGE_DIR, "config", "road_risk_calculator_params.yaml"),
        osp.join(PACKAGE_DIR, "config", "pose_predictor_params.yaml"),
        osp.join(PACKAGE_DIR, "config", "back_propagation_params.yaml"),
    )

    extremum_seeking_mpc = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace="/aiformula_planning",
        output="screen",
        emulate_tty=True,
        parameters=[*ROS_PARAM_CONFIG,
                    {"base_footprint_frame_id": FRAME_IDS["base_footprint"], },],
        remappings=[
            ('sub_road_l', TOPIC_NAMES["perception"]["lane_lines"]["left"]),
            ('sub_road_r', TOPIC_NAMES["perception"]["lane_lines"]["right"]),
            ('sub_odom', TOPIC_NAMES["sensing"]["odometry"]["gyro"]),
            ('sub_object_info', TOPIC_NAMES["perception"]["objects"]["info"])
        ],
    )

    return LaunchDescription([
        extremum_seeking_mpc,
    ])
