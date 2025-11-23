#ifndef PATH_TRACKER__MPC_NODE_HPP_
#define PATH_TRACKER__MPC_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Dense>
#include <vector>
#include <optional>

namespace path_tracker
{

struct State {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

struct Control {
    double v = 0.0;
    double w = 0.0;
};

class MPCNode : public rclcpp::Node
{
public:
    explicit MPCNode(const rclcpp::NodeOptions & options);

private:
    void on_path_received(const nav_msgs::msg::Path::SharedPtr msg);
    void control_loop();
    
    // MPC Solver functions
    std::vector<Control> solve_mpc(const State& current_state, const std::vector<State>& reference_trajectory);
    State step_model(const State& state, const Control& control, double dt);

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double dt_;
    int horizon_;
    double max_v_;
    double max_w_;
    double goal_tolerance_;

    // State
    nav_msgs::msg::Path::SharedPtr current_path_;
    std::vector<Control> predicted_controls_; // Warm start
};

} // namespace path_tracker

#endif // PATH_TRACKER__MPC_NODE_HPP_
