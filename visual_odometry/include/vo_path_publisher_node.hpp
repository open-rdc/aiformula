#include "vonav/path_publisher_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>

namespace vonav
{

Publisher::Publisher(const rclcpp::NodeOptions& options)
: Publisher("", options) {}

Publisher::Publisher(
    const std::string &name_space,
    const rclcpp::NodeOptions &options)
: rclcpp::Node("vo_path_publisher_node", name_space, options),
  freq_(declare_parameter("interval_ms", 100)),
  path_file_name_(declare_parameter("path_file_name", "vo_path"))
{
    initCommunication();
    loadCSV();
    path_msg_ = createPath(xs_, ys_);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(freq_),
        std::bind(&Publisher::loop, this));

    RCLCPP_INFO(this->get_logger(), "VO Path Publisher initialized");
}

/* ===================== ROS ===================== */

void Publisher::initCommunication()
{
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("vo_path", 10);

    file_path_ =
        ament_index_cpp::get_package_share_directory("vonav") +
        "/config/course_data/" + path_file_name_ + ".csv";
}

/* ===================== CSV ===================== */

void Publisher::loadCSV()
{
    std::ifstream file(file_path_);
    RCLCPP_INFO(this->get_logger(), "Loading CSV: %s", file_path_.c_str());

    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file");
        return;
    }

    std::string line;
    bool header_skipped = false;

    while (std::getline(file, line)) {
        if (!header_skipped) {  // skip header
            header_skipped = true;
            continue;
        }

        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> tokens;

        while (std::getline(ss, cell, ',')) {
            tokens.push_back(cell);
        }

        // CSV format: x, y, z, yaw
        double x = std::stod(tokens[0]);
        double y = std::stod(tokens[1]);

        xs_.push_back(x);
        ys_.push_back(y);
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Loaded %zu path points from CSV",
        xs_.size());
}

/* ===================== PATH ===================== */

nav_msgs::msg::Path
Publisher::createPath(
    const std::vector<double>& xs,
    const std::vector<double>& ys)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();

    for (size_t i = 0; i < xs.size(); ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now();
        pose.pose.position.x = xs[i];
        pose.pose.position.y = ys[i];
        pose.pose.position.z = 0.0;
        path.poses.push_back(pose);
    }

    return path;
}

/* ===================== LOOP ===================== */

void Publisher::loop()
{
    path_msg_.header.stamp = now();
    path_pub_->publish(path_msg_);
}

} // namespace vonav