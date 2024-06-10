#pragma once
#include "gnssnav/visibility_control.h"

#include <proj.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


namespace gnssnav{

class Publisher : public rclcpp ::Node{
public:
    Publisher();

    void loop(void);
    int freq_;

    GNSSNAV_PUBLIC
    explicit Publisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    GNSSNAV_PUBLIC
    explicit Publisher(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    std::string file_path_;
    std::string line_;
    std::vector<double> xs_;
    std::vector<double> ys_;
    std::vector<double> origin_xs_;
    std::vector<double> origin_ys_;

    std::string cell_;
    std::vector<std::string> tokens_;

    geometry_msgs::msg::PoseStamped pose_;
    geometry_msgs::msg::PoseStamped origin_pose_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr origin_publisher_;
    nav_msgs::msg::Path path_msg_;
    nav_msgs::msg::Path origin_path_msg_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::pair<double, double> convertGPStoUTM(double lat, double lon);
    std::vector<Eigen::Vector2d> interpolateSpline(const std::vector<double>& xs, const std::vector<double>& ys, int num_points);
    std::vector<Eigen::Vector2d> result_;

    nav_msgs::msg::Path setMsg(const std::vector<double>& xs, const std::vector<double>& ys);

    void initCommunication(void);
    void loadCSV(void);
    void _publisher_callback();
    void setInitPose(double x, double y);

    bool init_flag_;
    double base_x_;
    double base_y_;
    double step;

};

}  // namespace gnssnav
