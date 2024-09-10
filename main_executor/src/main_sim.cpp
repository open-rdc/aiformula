#include <rclcpp/rclcpp.hpp>

#include "controller/controller_node.hpp"
#include "gnssnav/path_publisher_node.hpp"
#include "gnssnav/follower_node.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);

    auto path_publisher_node = std::make_shared<gnssnav::Publisher>(nodes_option);
    auto follower_node = std::make_shared<gnssnav::Follower>(nodes_option);

    exec.add_node(path_publisher_node);
    exec.add_node(follower_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
