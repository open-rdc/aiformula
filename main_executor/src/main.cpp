#include <rclcpp/rclcpp.hpp>

#include "controller/controller_node.hpp"
#include "chassis_driver/chassis_driver_node.hpp"
#include "yolopnav/lane_line_publisher_node.hpp"
#include "obstacle_detector_ros2/obstacle_detector_node.hpp"
#include "path_tracker/pure_pursuit_node.hpp"
#include "gnssnav/path_publisher_node.hpp"
#include "gnssnav/follower_node.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);


    auto controller_node = std::make_shared<controller::Controller>(nodes_option);
    auto chassis_driver_node = std::make_shared<chassis_driver::ChassisDriver>(nodes_option);
    auto lane_line_node = std::make_shared<yolopnav::LaneLinePublisher>(nodes_option);
    auto obstacle_detector_node = std::make_shared<obstacle_detector_ros2::ObstacleDetectorNode>(nodes_option);
    auto path_tracker_node = std::make_shared<path_tracker::PurePursuit>(nodes_option);
    auto path_publisher_node = std::make_shared<gnssnav::Publisher>(nodes_option);
    auto follower_node = std::make_shared<gnssnav::Follower>(nodes_option);

    exec.add_node(controller_node);
    exec.add_node(chassis_driver_node);
    exec.add_node(lane_line_node);
    exec.add_node(obstacle_detector_node);
    exec.add_node(path_tracker_node);
    exec.add_node(path_publisher_node);
    exec.add_node(follower_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
