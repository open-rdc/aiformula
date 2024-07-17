#include <rclcpp/rclcpp.hpp>

#include "socketcan_interface/socketcan_interface_node.hpp"
#include "controller/controller_node.hpp"
#include "roboteq_driver/roboteq_driver_node.hpp"
#include "cybergear_interface/cybergear_interface_node.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);

    auto socketcan_node = std::make_shared<socketcan_interface::SocketcanInterface>(nodes_option);
    auto controller_node = std::make_shared<controller::Controller>(nodes_option);
    auto roboteq_driver_node = std::make_shared<roboteq_driver::RoboteqDriver>(nodes_option);
    auto cybergear_interface_node = std::make_shared<cybergear_interface::CybergearInterface>(nodes_option);

    exec.add_node(socketcan_node);
    exec.add_node(controller_node);
    exec.add_node(roboteq_driver_node);
    exec.add_node(cybergear_interface_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
