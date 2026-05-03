#include <rclcpp/rclcpp.hpp>

#include "zed_wrapper/zed_wrapper_node.hpp"
#include "controller/controller_node.hpp"
#include "chassis_driver/chassis_driver_node.hpp"
#include "frenet_planner/frenet_planner_node.hpp"
#include "path_tracker/pure_pursuit_node.hpp"
#include "localization/map_odom_tf_node.hpp"
#include "localization/odom_tf_node.hpp"
#include "localization/localization_node.hpp"
#include "motion_control/controller_server.hpp"
#include "lane_planner/lane_planner_node.hpp"
#include "local_planner/local_planner_server.hpp"
#include "vectormap_server/vectormap_server_node.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);

    // Read launch flags from the 'launch' node's parameters
    const bool use_zed =
        rclcpp::Node("launch", nodes_option).get_parameter("zed").as_bool();

    auto controller_node = std::make_shared<controller::Controller>(nodes_option);
    auto chassis_driver_node = std::make_shared<chassis_driver::ChassisDriver>(nodes_option);
    auto frenet_planner_node = std::make_shared<frenet_planner::FrenetPlannerNode>(nodes_option);
    auto path_tracker_node = std::make_shared<path_tracker::PurePursuit>(nodes_option);
    auto vectormap_server_node = std::make_shared<vectormap_server::VectormapServerNode>(nodes_option);
    auto localization_node =
        std::make_shared<localization::LocalizationNode>(nodes_option);
    auto odom_tf_node = std::make_shared<localization::OdomTfNode>(nodes_option);
    auto map_odom_tf_node = std::make_shared<localization::MapOdomTfNode>(nodes_option);
    auto lane_planner_node =
        std::make_shared<lane_planner::LanePlannerNode>(nodes_option);
    auto local_planner_server_node =
        std::make_shared<local_planner::LocalPlannerServer>(nodes_option);
    auto controller_server_node =
        std::make_shared<motion_control::ControllerServer>(nodes_option);

    if (use_zed) {
        exec.add_node(std::make_shared<zed_wrapper::ZedWrapperNode>(nodes_option));
    }
    exec.add_node(controller_node);
    exec.add_node(chassis_driver_node);
    // exec.add_node(frenet_planner_node);
    // exec.add_node(path_tracker_node);
    exec.add_node(vectormap_server_node);
    exec.add_node(localization_node);
    exec.add_node(odom_tf_node);
    exec.add_node(map_odom_tf_node);
    exec.add_node(lane_planner_node);
    exec.add_node(local_planner_server_node);
    exec.add_node(controller_server_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
