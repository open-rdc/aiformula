#include <rclcpp/rclcpp.hpp>

#include "controller/controller_node.hpp"
#include "chassis_driver/chassis_driver_node.hpp"
#include "frenet_planner/frenet_planner_node.hpp"
#include "path_tracker/pure_pursuit_node.hpp"
#include "vectormap_matching/map_odom_tf_node.hpp"
#include "vectormap_matching/odom_tf_node.hpp"
#include "vectormap_matching/vectormap_localization_node.hpp"
#include "vectormap_planner/vectormap_planner_node.hpp"
#include "vectormap_server/vectormap_server_node.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);


    auto controller_node = std::make_shared<controller::Controller>(nodes_option);
    auto chassis_driver_node = std::make_shared<chassis_driver::ChassisDriver>(nodes_option);
    auto frenet_planner_node = std::make_shared<frenet_planner::FrenetPlannerNode>(nodes_option);
    auto path_tracker_node = std::make_shared<path_tracker::PurePursuit>(nodes_option);
    auto vectormap_server_node = std::make_shared<vectormap_server::VectormapServerNode>(nodes_option);
    auto vectormap_localization_node =
        std::make_shared<vectormap_matching::VectormapLocalizationNode>(nodes_option);
    auto odom_tf_node = std::make_shared<vectormap_matching::OdomTfNode>(nodes_option);
    auto map_odom_tf_node = std::make_shared<vectormap_matching::MapOdomTfNode>(nodes_option);
    auto vectormap_planner_node =
        std::make_shared<vectormap_planner::VectormapPlannerNode>(nodes_option);

    exec.add_node(controller_node);
    exec.add_node(chassis_driver_node);
    // exec.add_node(frenet_planner_node);
    // exec.add_node(path_tracker_node);
    exec.add_node(vectormap_server_node);
    exec.add_node(vectormap_localization_node);
    exec.add_node(odom_tf_node);
    exec.add_node(map_odom_tf_node);
    exec.add_node(vectormap_planner_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
