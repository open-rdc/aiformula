#include <rclcpp/rclcpp.hpp>
#include "path_tracker/pure_pursuit_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<path_tracker::PurePursuit>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
