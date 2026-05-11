#include <rclcpp/rclcpp.hpp>

#include "zed_wrapper/zed_wrapper_node.hpp"
#include "controller/controller_node.hpp"
#include "chassis_driver/chassis_driver_node.hpp"
#include "localization/map_odom_tf_node.hpp"
#include "localization/odom_tf_node.hpp"
#include "localization/localization_node.hpp"
#include "motion_control/controller_server.hpp"
#include "lane_planner/lane_planner_node.hpp"
#include "local_planner/local_planner_server.hpp"
#include "object_detector/object_detector_node.hpp"
#include "vectormap_server/vectormap_server_node.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);

    // Read launch flags from the 'launch' node's parameters
    const bool use_zed = rclcpp::Node("launch", nodes_option).get_parameter("zed").as_bool();
    const bool use_sim = rclcpp::Node("launch", nodes_option).get_parameter("sim").as_bool();

    if (use_sim) {
        nodes_option.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    }

    auto controller_node = std::make_shared<controller::Controller>(nodes_option);
    auto chassis_driver_node = std::make_shared<chassis_driver::ChassisDriver>(nodes_option);
    auto vectormap_server_node = std::make_shared<vectormap_server::VectormapServerNode>(nodes_option);
    auto localization_node = std::make_shared<localization::LocalizationNode>(nodes_option);
    auto odom_tf_node = std::make_shared<localization::OdomTfNode>(nodes_option);
    auto map_odom_tf_node = std::make_shared<localization::MapOdomTfNode>(nodes_option);
    auto lane_planner_node = std::make_shared<lane_planner::LanePlannerNode>(nodes_option);
    auto local_planner_server_node = std::make_shared<local_planner::LocalPlannerServer>(nodes_option);
    auto controller_server_node = std::make_shared<motion_control::ControllerServer>(nodes_option);
    auto object_detector_node = std::make_shared<object_detector::ObjectDetectorNode>(nodes_option);

    std::shared_ptr<zed_wrapper::ZedWrapperNode> zed_wrapper_node;
    if (use_zed) {
        zed_wrapper_node = std::make_shared<zed_wrapper::ZedWrapperNode>(nodes_option);
        exec.add_node(zed_wrapper_node);
    }
    exec.add_node(controller_node);
    exec.add_node(chassis_driver_node);
    exec.add_node(vectormap_server_node);
    exec.add_node(localization_node);
    exec.add_node(odom_tf_node);
    exec.add_node(map_odom_tf_node);
    exec.add_node(lane_planner_node);
    exec.add_node(local_planner_server_node);
    exec.add_node(controller_server_node);
    exec.add_node(object_detector_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
