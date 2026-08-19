#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <memory>
#include <string>

#include "road_detector/lane_segmenter.hpp"
#include "road_detector/letterbox.hpp"
#include "road_detector/visibility_control.h"

namespace road_detector {

class RoadDetectorNode : public rclcpp::Node {
   public:
    ROAD_DETECTOR_PUBLIC explicit RoadDetectorNode(const rclcpp::NodeOptions& options);

   private:
    // intra-process 化(別ブランチ)で余計なコピーを生まないよう ConstSharedPtr で受ける
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void publish_mask(const cv::Mat& mask, const std_msgs::msg::Header& header);
    void publish_visualize(
        const cv::Mat& image, const cv::Mat& mask, const std_msgs::msg::Header& header);

    cv::Size          image_size_;
    float             mask_threshold_ = 0.5F;
    LetterboxGeometry geometry_;

    std::unique_ptr<LaneSegmenter> segmenter_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    mask_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    visualize_publisher_;
};

}  // namespace road_detector
