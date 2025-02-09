#include "imagenav/imagenav_node.hpp"

#include "utilities/utils.hpp"
#include "cmath"

namespace imagenav{

ImageNav::ImageNav(const rclcpp::NodeOptions &options) : ImageNav("", options) {}

ImageNav::ImageNav(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("imagenav_node", name_space, options),
interval_ms(get_parameter("interval_ms").as_int()),
pid(get_parameter("interval_ms").as_int()),
linear_max_(get_parameter("max_linear_vel").as_double()),
angular_max_(get_parameter("max_angular_vel").as_double())
{
    autonomous_flag_subscriber_ = this->create_subscription<std_msgs::msg::Bool>("/autonomous", 10, std::bind(&ImageNav::autonomousFlagCallback, this, std::placeholders::_1));
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/zed/zed_node/rgb/image_rect_color", 10, std::bind(&ImageNav::ImageCallback, this, std::placeholders::_1));
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(interval_ms),
        std::bind(&ImageNav::ImageNavigation, this));

    pid.gain(get_parameter("p_gain").as_double(), get_parameter("i_gain").as_double(), get_parameter("d_gain").as_double());
}

void ImageNav::autonomousFlagCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    autonomous_flag_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Autonomous flag updated to: %s", autonomous_flag_ ? "true" : "false");
}

void ImageNav::ImageCallback(const sensor_msgs::msg::Image::SharedPtr img)
{
    cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img, img->encoding);
    if(cv_img->image.empty() || !autonomous_flag_)  return;

    cv::Mat detect_img = line.detectLine(cv_img->image);

    // 白線の開始位置（x座標）を検出
    std::vector<int> estimate_x = line.estimateLinePosition(detect_img);

    if(estimate_x[0] > estimate_x[1])
    {
        left_line_x = estimate_x[0];
        right_line_x = estimate_x[1];
    }else{
        right_line_x = estimate_x[0];
        left_line_x = estimate_x[1];
    }

    // スライドウィンドウ法で窓の中心点を抽出
    std::vector<cv::Point> left_points = line.SlideWindowMethod(detect_img, left_line_x);
    std::vector<cv::Point> right_points = line.SlideWindowMethod(detect_img, right_line_x);

    for(size_t i=0; i < left_points.size(); ++i)
    {
        int x = (left_points[i].x + right_points[i].x) / 2;
        int y = (left_points[i].y + right_points[i].y) / 2;

        center_points.push_back(cv::Point(x, y));
    }

    cv::Mat point_img = line.PointVisualizar(cv_img->image, left_points);
    point_img = line.PointVisualizar(point_img, right_points);
    point_img = line.PointVisualizar(point_img, center_points);

    cv::imshow("image data", cv_img->image);
    cv::waitKey(1);
}

void ImageNav::ImageNavigation(void)
{
    const int image_rows=240;
    const int image_cols=240;

    if(center_points.empty() || !autonomous_flag_)
    {
        center_points.clear();
        return;
    }

    // center_pointsに向かうように移動<-[4]は適当
    int dx = image_rows - center_points[4].x;
    int dy = image_cols - center_points[4].y;

    center_points.clear();

    double angle = std::atan2(dx, dy); // 座標変換のため, dx,dyを入れ替え

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_max_;
    cmd_vel.angular.z = pid.cycle(angle);

    cmd_pub_->publish(cmd_vel);
}

}  // namespace imagenav
