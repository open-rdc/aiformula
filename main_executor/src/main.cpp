#include <rclcpp/rclcpp.hpp>

#include "socketcan_interface/socketcan_interface_node.hpp"
#include "controller/controller_node.hpp"
#include "chassis_driver/chassis_driver_node.hpp"
#include "gnssnav/path_publisher_node.hpp"
#include "gnssnav/follower_node.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);

    bool sim_flag = false;
    for(int i = 0; i < argc; ++i){
        if(std::string(argv[i])=="--sim-flag"){
            if(std::string(argv[i+1])=="true") sim_flag = true;
            break;
        }
    }

    auto socketcan_node = std::make_shared<socketcan_interface::SocketcanInterface>(nodes_option);
    auto controller_node = std::make_shared<controller::Controller>(nodes_option);
    auto chassis_driver_node = std::make_shared<chassis_driver::ChassisDriver>(nodes_option);
    auto path_publisher_node = std::make_shared<gnssnav::Publisher>(nodes_option);
    auto follower_node = std::make_shared<gnssnav::Follower>(nodes_option);

    if(sim_flag){}
    if(not sim_flag){
        exec.add_node(socketcan_node);
    }
    exec.add_node(controller_node);
    exec.add_node(chassis_driver_node);
    exec.add_node(path_publisher_node);
    exec.add_node(follower_node);

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
