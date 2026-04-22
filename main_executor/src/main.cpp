#include <rclcpp/rclcpp.hpp>

#include "controller/controller_node.hpp"
#include "chassis_driver/chassis_driver_node.hpp"
#include "frenet_planner/frenet_planner_node.hpp"
#include "path_tracker/pure_pursuit_node.hpp"

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

    exec.add_node(controller_node);
    exec.add_node(chassis_driver_node);
    exec.add_node(frenet_planner_node);
    exec.add_node(path_tracker_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
