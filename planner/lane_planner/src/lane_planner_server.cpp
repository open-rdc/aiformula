#include "lane_planner/lane_planner_server.hpp"

#include <stdexcept>

#include "lane_planner/mapless_lane_planner_plugin.hpp"
#include "lane_planner/vectormap_lane_planner_plugin.hpp"

namespace lane_planner
{

LanePlannerServer::LanePlannerServer(const rclcpp::NodeOptions & options)
: LanePlannerServer("", options)
{
}

LanePlannerServer::LanePlannerServer(
    const std::string & name_space,
    const rclcpp::NodeOptions & options)
: rclcpp::Node("lane_planner_node", name_space, options),
  update_period_ms_(get_parameter("update_period_ms").as_int()),
  qos_(rclcpp::QoS(10))
{
    if (update_period_ms_ <= 0) {
        throw std::invalid_argument("update_period_ms must be greater than 0");
    }

    const std::string mode = get_parameter("lane_planner_mode").as_string();

    if (mode == "vectormap") {
        plugin_ = std::make_unique<VectormapLanePlannerPlugin>();
    } else if (mode == "mapless") {
        plugin_ = std::make_unique<MaplessLanePlannerPlugin>();
    } else {
        throw std::invalid_argument("lane_planner_mode must be 'vectormap' or 'mapless': " + mode);
    }

    plugin_->initialize(*this);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        get_parameter("localization_pose_topic").as_string(),
        qos_,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            pose_callback(msg);
        });

    nav_cmd_sub_ = create_subscription<std_msgs::msg::String>(
        get_parameter("nav_cmd_topic").as_string(),
        qos_,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            nav_cmd_callback(msg);
        });

    if (mode == "mapless") {
        road_segments_sub_ = create_subscription<mapless_planning_msgs::msg::RoadSegments>(
            get_parameter("road_segments_topic").as_string(),
            qos_,
            [this](const mapless_planning_msgs::msg::RoadSegments::SharedPtr msg) {
                road_segments_callback(msg);
            });
        mission_lanes_pub_ = create_publisher<mapless_planning_msgs::msg::MissionLanesStamped>(
            get_parameter("mission_lanes_topic").as_string(), qos_);
    }

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        get_parameter("global_path_topic").as_string(), qos_);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(update_period_ms_),
        std::bind(&LanePlannerServer::timer_callback, this));

    RCLCPP_INFO(get_logger(), "LanePlannerServer started in '%s' mode", mode.c_str());
}

void LanePlannerServer::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    plugin_->set_pose(*msg);
}

void LanePlannerServer::nav_cmd_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_ERROR(get_logger(), "nav_cmd message must not be null");
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    plugin_->set_nav_command(msg->data);
}

void LanePlannerServer::road_segments_callback(
    const mapless_planning_msgs::msg::RoadSegments::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_ERROR(get_logger(), "RoadSegments message must not be null");
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    plugin_->set_road_segments(*msg);
}

void LanePlannerServer::timer_callback()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    const auto path = plugin_->compute_path(now());
    if (!path) {
        return;
    }
    path_pub_->publish(*path);

    if (mission_lanes_pub_) {
        const auto ml = plugin_->get_mission_lanes();
        if (ml) {
            mission_lanes_pub_->publish(*ml);
        }
    }
}

}  // namespace lane_planner
