#include <rclcpp/rclcpp.hpp>

#include "zed_wrapper/zed_wrapper_node.hpp"
#include "controller/controller_node.hpp"
#include "chassis_driver/chassis_driver_node.hpp"
#include "lane_line_publisher/lane_line_publisher_node.hpp"
#include "lane_line_publisher/vectormap_visualizer_node.hpp"
#include "ekf_localizer/map_odom_tf_node.hpp"
#include "ekf_localizer/odom_tf_node.hpp"
#include "ekf_localizer/ekf_localizer_node.hpp"
#include "pose_estimater/pose_estimater_node.hpp"
#include "trajectory_follower/controller_server.hpp"
#include "mission_planner/mission_planner_node.hpp"
#include "local_planner/local_planner_server.hpp"
#include "object_detector/object_detector_node.hpp"
#include "vectormap_server/vectormap_server_node.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);
    const bool use_sim = rclcpp::Node("launch", nodes_option).get_parameter("sim").as_bool();

    if (use_sim) {
        nodes_option.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    }

    auto controller_node = std::make_shared<controller::Controller>(nodes_option);
    auto chassis_driver_node = std::make_shared<chassis_driver::ChassisDriver>(nodes_option);
    auto vectormap_server_node = std::make_shared<vectormap_server::VectormapServerNode>(nodes_option);
    auto lane_line_publisher_node = std::make_shared<lane_line_publisher::LaneLinePublisherNode>(nodes_option);
    auto vectormap_visualizer_node = std::make_shared<lane_line_publisher::VectormapVisualizerNode>(nodes_option);
    auto pose_estimater_node = std::make_shared<pose_estimater::PoseEstimaterNode>(nodes_option);
    auto ekf_localizer_node = std::make_shared<ekf_localizer::EkfLocalizerNode>(nodes_option);
    auto odom_tf_node = std::make_shared<ekf_localizer::OdomTfNode>(nodes_option);
    auto map_odom_tf_node = std::make_shared<ekf_localizer::MapOdomTfNode>(nodes_option);
    auto mission_planner_node = std::make_shared<mission_planner::MissionPlannerNode>(nodes_option);
    auto local_planner_server_node = std::make_shared<local_planner::LocalPlannerServer>(nodes_option);
    auto controller_server_node = std::make_shared<trajectory_follower::ControllerServer>(nodes_option);
    // auto object_detector_node = std::make_shared<object_detector::ObjectDetectorNode>(nodes_option);

#ifdef ENABLE_ZED
    std::shared_ptr<zed_wrapper::ZedWrapperNode> zed_wrapper_node;
    if (!use_sim) {
        zed_wrapper_node = std::make_shared<zed_wrapper::ZedWrapperNode>(nodes_option);
        exec.add_node(zed_wrapper_node);
    }
#endif
    exec.add_node(controller_node);
    exec.add_node(chassis_driver_node);
    exec.add_node(vectormap_server_node);
    exec.add_node(lane_line_publisher_node);
    exec.add_node(vectormap_visualizer_node);
    exec.add_node(pose_estimater_node);
    exec.add_node(ekf_localizer_node);
    exec.add_node(odom_tf_node);
    exec.add_node(map_odom_tf_node);
    exec.add_node(mission_planner_node);
    exec.add_node(local_planner_server_node);
    exec.add_node(controller_server_node);
    // exec.add_node(object_detector_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
