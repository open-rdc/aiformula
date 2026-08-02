#include "local_planner/local_planner_server.hpp"

#include <cmath>
#include <stdexcept>

namespace local_planner
{

bool is_stamp_fresh(
    const rclcpp::Time& now,
    const rclcpp::Time& stamp,
    const double timeout_s)
{
    return (now - stamp).seconds() <= timeout_s;
}

LocalPlannerServer::LocalPlannerServer(const rclcpp::NodeOptions& options)
: LocalPlannerServer("", options)
{
}

LocalPlannerServer::LocalPlannerServer(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("local_planner_server_node", name_space, options),
  plugin_loader_("local_planner", "local_planner::LocalPlannerPlugin"),
  update_period_ms_(get_parameter("update_period_ms").as_int()),
  input_timeout_s_(get_parameter("input_timeout_s").as_double()),
  qos_(rclcpp::QoS(10))
{
    if (update_period_ms_ <= 0) {
        throw std::invalid_argument("update_period_ms must be greater than 0");
    }
    if (!std::isfinite(input_timeout_s_) || input_timeout_s_ <= 0.0) {
        throw std::invalid_argument("input_timeout_s must be finite and greater than 0");
    }

    const auto plugin_name = get_parameter("local_planner_plugin").as_string();
    if (plugin_name.empty()) {
        throw std::invalid_argument("local_planner_plugin must not be empty");
    }
    plugin_ = plugin_loader_.createSharedInstance(plugin_name);
    plugin_->initialize(get_logger(), get_clock(), get_node_parameters_interface());

    global_path_subscription_ = create_subscription<nav_msgs::msg::Path>(
        "/planner/global_path",
        qos_,
        std::bind(&LocalPlannerServer::global_path_callback, this, std::placeholders::_1));
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/localization/pose",
        qos_,
        std::bind(&LocalPlannerServer::pose_callback, this, std::placeholders::_1));
    velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/vectornav/velocity_body",
        qos_,
        std::bind(&LocalPlannerServer::velocity_callback, this, std::placeholders::_1));
    objects_subscription_ = create_subscription<object_detection_msgs::msg::ObjectInfoArray>(
        "/perception/objects",
        qos_,
        std::bind(&LocalPlannerServer::objects_callback, this, std::placeholders::_1));

    local_path_publisher_ = create_publisher<nav_msgs::msg::Path>("/planner/local_path", qos_);
    timer_ = create_wall_timer(
        std::chrono::milliseconds(update_period_ms_),
        std::bind(&LocalPlannerServer::timer_callback, this));
}

void LocalPlannerServer::global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (!msg || msg->poses.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "global_pathが空またはnullのため無視する");
        return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    plugin_->setGlobalPath(*msg);
}

void LocalPlannerServer::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_pose_ = msg;
}

void LocalPlannerServer::velocity_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_velocity_ = msg;
}

void LocalPlannerServer::objects_callback(
    const object_detection_msgs::msg::ObjectInfoArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_objects_ = msg;
}

void LocalPlannerServer::timer_callback()
{
    std::optional<nav_msgs::msg::Path> result;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!latest_pose_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "自己位置の受信待ち");
            return;
        }

        const rclcpp::Time current_time = this->now();
        const rclcpp::Time pose_stamp(
            latest_pose_->header.stamp, current_time.get_clock_type());
        if (!is_stamp_fresh(current_time, pose_stamp, input_timeout_s_)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "自己位置が%.1fs以上古いためplanningをスキップする", input_timeout_s_);
            return;
        }

        geometry_msgs::msg::TwistWithCovarianceStamped velocity;
        if (latest_velocity_) {
            velocity = *latest_velocity_;
        }

        const object_detection_msgs::msg::ObjectInfoArray* objects_ptr = nullptr;
        if (latest_objects_) {
            const rclcpp::Time objects_stamp(
                latest_objects_->header.stamp, current_time.get_clock_type());
            if (is_stamp_fresh(current_time, objects_stamp, input_timeout_s_)) {
                objects_ptr = latest_objects_.get();
            } else {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "障害物情報が%.1fs以上古いため障害物なしとして扱う", input_timeout_s_);
            }
        }

        result = plugin_->computeLocalPath(*latest_pose_, velocity, objects_ptr);
    }

    if (!result) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "local pathが空のためpublishをスキップする");
        return;
    }
    local_path_publisher_->publish(*result);
}

}
