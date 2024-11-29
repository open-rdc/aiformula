#include "line_publisher/line_publisher_node.hpp"

namespace line_publisher{

LinePublisherNode::LinePublisherNode(const rclcpp::NodeOptions &options) : LinePublisherNode("", options) {}

LinePublisherNode::LinePublisherNode(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("line_publisher_node", name_space, options),
ksize(get_parameter("ksize").as_int()),
threshold(get_parameter("threshold").as_int()),
min_th(get_parameter("min_th").as_int()),
max_th(get_parameter("max_th").as_int()),
interval_ms(get_parameter("interval_ms").as_int())
{
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&LinePublisherNode::ImageCallback, this, std::placeholders::_1));
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/line_detection/image", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms),
        std::bind(&LinePublisherNode::LineDetector, this)
    );
}

void LinePublisherNode::LineDetector()
{
    if(cv_img->image.empty())    return;

    cv::Mat gray_img, gaussian_img, edge_img;
    cv::cvtColor(cv_img->image, gray_img, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray_img, gray_img);
    cv::GaussianBlur(gray_img, gaussian_img, cv::Size(ksize, ksize), 5);
    cv::Canny(gaussian_img, edge_img, min_th, max_th, 3);

    std::vector <cv::Vec4i> line;
    cv::HoughLinesP(edge_img, line, 1, CV_PI / 180, threshold, 0.2, 5);

    for (size_t i = 0; i < line.size(); i++){
        cv::line(cv_img->image, cv::Point(line[i][0], line[i][1]),
                cv::Point(line[i][2], line[i][3]), cv::Scalar(0, 0, 255),
                20, 8);

        // std::cerr << line[i][0] << ", " << line[i][1] << ", " << line[i][2] << ", " << line[i][3] << std::endl; 
    }

    cv::drawMarker(cv_img->image, cv::Point(0, 0), cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 50, 10);
    cv::drawMarker(cv_img->image, cv::Point(1920, 1080), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 50, 10);
    cv::drawMarker(cv_img->image, cv::Point(1920, 0), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 50, 10);
    cv::drawMarker(cv_img->image, cv::Point(0, 1080), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 50, 10);

    // 左車線
    cv::drawMarker(cv_img->image, cv::Point(150, 1080), cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 50, 10);
    cv::drawMarker(cv_img->image, cv::Point(760, 540), cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 50, 10);
    // 右車線
    cv::drawMarker(cv_img->image, cv::Point(1650, 1080), cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 50, 10);
    cv::drawMarker(cv_img->image, cv::Point(1140, 540), cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 50, 10);

    sensor_msgs::msg::Image::SharedPtr ros_img = cv_bridge::CvImage(cv_img->header, "bgr8", cv_img->image).toImageMsg();

    image_pub_->publish(*ros_img);
}

}  // namespace line_publisher
