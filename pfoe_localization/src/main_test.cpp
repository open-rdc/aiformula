#include <rclcpp/rclcpp.hpp>
#include "pfoe_localization/pfoe_node.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::NodeOptions nodes_option;

    auto pfoe_localization_node = std::make_shared<pfoe_localization::PfoeNode>(nodes_option);
    exec.add_node(pfoe_localization_node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}