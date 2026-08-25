#include "lane_line_publisher/vectormap_visualizer_node.hpp"

#include <camera_utility/camera_parameters.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2/exceptions.h>

#include "lane_line_publisher/vectormap_projection.hpp"

namespace lane_line_publisher
{

VectormapVisualizerNode::VectormapVisualizerNode(const rclcpp::NodeOptions& options)
: VectormapVisualizerNode("", options)
{
}

VectormapVisualizerNode::VectormapVisualizerNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options)
: rclcpp::Node("vectormap_visualizer_node", name_space, options),
  camera_intrinsics_(camera_utility::getCameraIntrinsics(*this)),
  base_T_camera_(camera_utility::getBaseTCamera(*this))
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/zed/zed_node/rgb/image_rect_color", rclcpp::QoS(10),
        std::bind(&VectormapVisualizerNode::image_callback, this, std::placeholders::_1));
    rclcpp::SubscriptionOptions no_intra_process_options;
    no_intra_process_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;

    vector_map_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/vector_map/visualize", rclcpp::QoS(1).transient_local(),
        std::bind(&VectormapVisualizerNode::vector_map_callback, this, std::placeholders::_1),
        no_intra_process_options);

    vectormap_visualize_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/perception/vectormap_visualize", rclcpp::QoS(10));
}

void VectormapVisualizerNode::vector_map_callback(
    const visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(vector_map_mutex_);
    latest_vector_map_markers_ = *msg;
}

void VectormapVisualizerNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    if (vectormap_visualize_publisher_->get_subscription_count() == 0) {
        return;
    }

    visualization_msgs::msg::MarkerArray vector_map_markers;
    {
        std::lock_guard<std::mutex> lock(vector_map_mutex_);
        vector_map_markers = latest_vector_map_markers_;
    }
    if (vector_map_markers.markers.empty()) {
        return;
    }

    geometry_msgs::msg::TransformStamped base_to_map;
    try {
        base_to_map = tf_buffer_->lookupTransform("base_link", "map", tf2::TimePointZero);
    } catch (const tf2::TransformException& error) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "vectormap_visualizer skipped: %s", error.what());
        return;
    }

    const auto& translation = base_to_map.transform.translation;
    const auto& rotation = base_to_map.transform.rotation;
    const tf2::Transform base_T_map(
        tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        tf2::Vector3(translation.x, translation.y, translation.z));

    try {
        auto image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

        const auto projected_line_strings = project_vector_map_markers(
            vector_map_markers, base_T_map, base_T_camera_, camera_intrinsics_);
        draw_projected_line_strings(image, projected_line_strings);

        auto visualize_msg = std::make_unique<sensor_msgs::msg::Image>();
        cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, image)
            .toImageMsg(*visualize_msg);
        vectormap_visualize_publisher_->publish(std::move(visualize_msg));
    } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "vectormap_visualizer skipped: %s", error.what());
    }
}

}
