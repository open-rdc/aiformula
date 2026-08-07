#include "local_planner/local_planner_server.hpp"

namespace local_planner
{

LocalPlannerServer::LocalPlannerServer(const rclcpp::NodeOptions& options)
: LocalPlannerServer("", options)
{
}

LocalPlannerServer::LocalPlannerServer(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("local_planner_server_node", name_space, options),
  plugin_loader_("local_planner", "local_planner::LocalPlannerPlugin"),
  interval_ms_(get_parameter("interval_ms").as_int()),
  qos_(rclcpp::QoS(10))
{
    const auto plugin_name = get_parameter("local_planner_plugin").as_string();
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
        std::chrono::milliseconds(interval_ms_),
        std::bind(&LocalPlannerServer::timer_callback, this));
}

void LocalPlannerServer::global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    plugin_->setGlobalPath(*msg);
}

void LocalPlannerServer::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    pose_ = msg;
}

void LocalPlannerServer::velocity_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    velocity_ = msg;
}

void LocalPlannerServer::objects_callback(
    const object_detection_msgs::msg::ObjectInfoArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    objects_ = msg;
}

void LocalPlannerServer::timer_callback()
{
    std::optional<nav_msgs::msg::Path> result;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!pose_) {
            RCLCPP_DEBUG(get_logger(), "自己位置の受信待ち");
            return;
        }

        geometry_msgs::msg::TwistWithCovarianceStamped velocity;
        if (velocity_) {
            velocity = *velocity_;
        }

        result = plugin_->computeLocalPath(*pose_, velocity, objects_.get());
    }

    if (!result) {
        RCLCPP_DEBUG(get_logger(), "ローカル経路の計算に失敗しました");
        return;
    }
    local_path_publisher_->publish(*result);
}

}
