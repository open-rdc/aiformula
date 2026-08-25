#include "road_detector/road_detector_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <utility>

namespace road_detector {

RoadDetectorNode::RoadDetectorNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("road_detector_node", options),
      image_size_(0, 0),
      mask_threshold_(static_cast<float>(get_parameter("mask_threshold").as_double()))
{
    mask_publisher_      = create_publisher<sensor_msgs::msg::Image>("/perception/lane_mask", rclcpp::QoS(10));
    visualize_publisher_ = create_publisher<sensor_msgs::msg::Image>("/perception/lane_mask_visualize", rclcpp::QoS(10));

    const std::string engine_path = ament_index_cpp::get_package_share_directory("road_detector") + "/weights/yolopv2.engine";
    segmenter_ = std::make_unique<LaneSegmenter>(engine_path);

    image_subscription_ = create_subscription<sensor_msgs::msg::Image>("/zed/zed_node/rgb/image_rect_color", rclcpp::QoS(1).best_effort(), std::bind(&RoadDetectorNode::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "road_detector 起動 (engine=%s, 入力=%dx%d)", engine_path.c_str(), segmenter_->input_size().width, segmenter_->input_size().height);
}

void RoadDetectorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    const cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

    // 受信画像の解像度を作業解像度とする（配信元の解像度に自動追従）
    const cv::Size frame_size(frame.cols, frame.rows);
    if (image_size_ != frame_size) {
        // 例外を握らないと単一プロセス構成の全ノードが道連れになる
        try {
            geometry_ = compute_letterbox_geometry(frame_size, segmenter_->input_size());
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "letterbox設定に失敗しました: %s", e.what());
            return;
        }
        image_size_ = frame_size;
    }
    const cv::Mat padded     = apply_letterbox(frame, segmenter_->input_size(), geometry_);
    const cv::Mat confidence = segmenter_->infer(padded);
    if (confidence.empty()) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 1000, "推論に失敗しました: %s",
            segmenter_->last_error().c_str());
        return;
    }
    const cv::Mat mask = to_lane_mask(confidence, image_size_, geometry_, mask_threshold_);

    publish_mask(mask, msg->header);
    if (visualize_publisher_->get_subscription_count() > 0) {
        publish_visualize(frame, mask, msg->header);
    }
}

void RoadDetectorNode::publish_mask(const cv::Mat& mask, const std_msgs::msg::Header& header)
{
    auto message = std::make_unique<sensor_msgs::msg::Image>();
    cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, mask).toImageMsg(*message);
    mask_publisher_->publish(std::move(message));
}

void RoadDetectorNode::publish_visualize(
    const cv::Mat& image, const cv::Mat& mask, const std_msgs::msg::Header& header)
{
    cv::Mat overlay = image.clone();
    overlay.setTo(cv::Scalar(0, 0, 255), mask);

    auto message = std::make_unique<sensor_msgs::msg::Image>();
    cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, overlay).toImageMsg(*message);
    visualize_publisher_->publish(std::move(message));
}

}  // namespace road_detector
