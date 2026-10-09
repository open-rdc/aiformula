#include "local_planner/local_planner_server.hpp"

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer_interface.hpp>

namespace local_planner
{
namespace
{
constexpr const char* planning_frame = "odom";
constexpr const char* vehicle_frame = "base_link";

// 経路をodomへ変換するときのTF参照時刻。
// 車体フレームの経路は観測時刻の姿勢に固定する(最新姿勢で貼り直すと古い経路が車に追従してしまう)。
// 固定フレーム(map)の経路は最新のmap->odom補正を使う。
tf2::TimePoint path_lookup_time(const nav_msgs::msg::Path& path)
{
    if (path.header.frame_id == vehicle_frame) {
        return tf2_ros::fromMsg(path.header.stamp);
    }
    return tf2::TimePointZero;
}

nav_msgs::msg::Path transform_path(
    const nav_msgs::msg::Path& path,
    const geometry_msgs::msg::TransformStamped& transform)
{
    nav_msgs::msg::Path transformed;
    transformed.header.stamp = path.header.stamp;
    transformed.header.frame_id = transform.header.frame_id;
    transformed.poses.reserve(path.poses.size());
    for (const auto& pose : path.poses) {
        geometry_msgs::msg::PoseStamped transformed_pose;
        tf2::doTransform(pose, transformed_pose, transform);
        transformed_pose.header = transformed.header;
        transformed.poses.push_back(transformed_pose);
    }
    return transformed;
}

object_detection_msgs::msg::ObjectInfoArray transform_objects(
    const object_detection_msgs::msg::ObjectInfoArray& objects,
    const geometry_msgs::msg::TransformStamped& transform)
{
    object_detection_msgs::msg::ObjectInfoArray transformed = objects;
    transformed.header.frame_id = transform.header.frame_id;
    for (auto& object : transformed.objects) {
        geometry_msgs::msg::Point point;
        point.x = object.x;
        point.y = object.y;
        point.z = 0.0;
        geometry_msgs::msg::Point transformed_point;
        tf2::doTransform(point, transformed_point, transform);
        object.x = static_cast<float>(transformed_point.x);
        object.y = static_cast<float>(transformed_point.y);
    }
    return transformed;
}

geometry_msgs::msg::PoseWithCovarianceStamped pose_from_transform(
    const geometry_msgs::msg::TransformStamped& transform)
{
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header = transform.header;
    pose.pose.pose.position.x = transform.transform.translation.x;
    pose.pose.pose.position.y = transform.transform.translation.y;
    pose.pose.pose.position.z = transform.transform.translation.z;
    pose.pose.pose.orientation = transform.transform.rotation;
    return pose;
}
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
  interval_ms_(get_parameter("interval_ms").as_int()),
  qos_(rclcpp::QoS(10))
{
    const auto plugin_name = get_parameter("local_planner_plugin").as_string();
    plugin_ = plugin_loader_.createSharedInstance(plugin_name);
    plugin_->initialize(get_logger(), get_clock(), get_node_parameters_interface());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    global_path_subscription_ = create_subscription<nav_msgs::msg::Path>("/planner/global_path", qos_, std::bind(&LocalPlannerServer::global_path_callback, this, std::placeholders::_1));
    velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/vectornav/velocity_body", qos_, std::bind(&LocalPlannerServer::velocity_callback, this, std::placeholders::_1));
    objects_subscription_ = create_subscription<object_detection_msgs::msg::ObjectInfoArray>("/perception/objects", rclcpp::SensorDataQoS().keep_last(1), std::bind(&LocalPlannerServer::objects_callback, this, std::placeholders::_1));

    local_path_publisher_ = create_publisher<nav_msgs::msg::Path>("/planner/local_path", qos_);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(interval_ms_),
        std::bind(&LocalPlannerServer::timer_callback, this));
}

void LocalPlannerServer::global_path_callback(const nav_msgs::msg::Path::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    global_path_ = msg;
}

void LocalPlannerServer::velocity_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    velocity_ = msg;
}

void LocalPlannerServer::objects_callback(
    const object_detection_msgs::msg::ObjectInfoArray::ConstSharedPtr msg)
{
    const auto odom_T_objects = lookup_to_odom(msg->header.frame_id, tf2::TimePointZero);
    if (!odom_T_objects) {
        return;
    }
    object_detection_msgs::msg::ObjectInfoArray objects_in_odom = transform_objects(*msg, *odom_T_objects);

    std::lock_guard<std::mutex> lock(data_mutex_);
    objects_ = std::move(objects_in_odom);
}

void LocalPlannerServer::timer_callback()
{
    const auto odom_T_base = lookup_to_odom(vehicle_frame, tf2::TimePointZero);
    if (!odom_T_base) {
        return;
    }
    const geometry_msgs::msg::PoseWithCovarianceStamped ego_pose = pose_from_transform(*odom_T_base);

    std::optional<nav_msgs::msg::Path> result;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!global_path_) {
            RCLCPP_DEBUG(get_logger(), "global pathの受信待ち");
            return;
        }
        // 毎周期変換し直す。map->odomは自己位置補正で動き続け、mission_plannerは周回経路を一度しか
        // publishしないため、受信時に一度だけ変換すると経路が古い補正に固定される
        const auto odom_T_path = lookup_to_odom(global_path_->header.frame_id, path_lookup_time(*global_path_));
        if (odom_T_path) {
            plugin_->setGlobalPath(transform_path(*global_path_, *odom_T_path));
        }

        geometry_msgs::msg::TwistWithCovarianceStamped velocity;
        if (velocity_) {
            velocity = *velocity_;
        }

        result = plugin_->computeLocalPath(ego_pose, velocity, objects_ ? &*objects_ : nullptr);
    }

    if (!result) {
        RCLCPP_DEBUG(get_logger(), "ローカル経路の計算に失敗しました");
        return;
    }
    local_path_publisher_->publish(std::make_unique<nav_msgs::msg::Path>(std::move(*result)));
}

std::optional<geometry_msgs::msg::TransformStamped> LocalPlannerServer::lookup_to_odom(
    const std::string& source_frame, const tf2::TimePoint& time)
{
    try {
        return tf_buffer_->lookupTransform(planning_frame, source_frame, time);
    } catch (const tf2::TransformException& error) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "%s -> %s のTFが引けないためスキップします: %s", planning_frame, source_frame.c_str(), error.what());
        return std::nullopt;
    }
}

}
