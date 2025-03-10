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
angular_max_(get_parameter("max_angular_vel").as_double()),
visualize_flag_(get_parameter("visualize_flag").as_bool())
{
    autonomous_flag_subscriber_ = this->create_subscription<std_msgs::msg::Bool>("/autonomous", 10, std::bind(&ImageNav::autonomousFlagCallback, this, std::placeholders::_1));
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/zed/zed_node/rgb/image_rect_color", 10, std::bind(&ImageNav::ImageCallback, this, std::placeholders::_1));
    depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/zed/zed_node/depth/depth_registered", 10, std::bind(&ImageNav::DepthImageCallback, this, std::placeholders::_1));
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/imagenav/image", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/imagenav/path", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/imagenav/marker", 10);
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
    cv_img = cv_bridge::toCvCopy(img, img->encoding);

    if(visualize_flag_)
    {
        cv::imshow("img", cv_img->image);
        cv::waitKey(1);
    }
}

void ImageNav::DepthImageCallback(const sensor_msgs::msg::Image::SharedPtr img)
{
    cv_depth_img = cv_bridge::toCvCopy(img, img->encoding);

    if(visualize_flag_)
    {
        cv::imshow("depth", cv_depth_img->image);
        cv::waitKey(1);
    }
}

std::vector<geometry_msgs::msg::Point> ImageNav::interpolateSpline(const std::vector<geometry_msgs::msg::Point>& center_points, int num_points)
{
    double step=0;
    typedef Eigen::Spline<double,2> Spline2d;
    std::vector<geometry_msgs::msg::Point> result;
    geometry_msgs::msg::Point point;
    Eigen::Matrix<double, Eigen::Dynamic, 2> points(center_points.size(), 2);

    for(size_t i=0; i < center_points.size(); i++)
    {
        points(i, 0) = center_points[i].x;
        points(i, 1) = center_points[i].y;
    }
    const Spline2d spline = Eigen::SplineFitting<Spline2d>::Interpolate(points.transpose(), 2);

    if(num_points > 1)
        step = 1.0 / (num_points-1);
    for(int i=0; i < num_points; i++)
    {
        double u = i*step;
        Eigen::Vector2d pt = spline(u);

        point.x = pt.x();
        point.y = pt.y();
        point.z = 0.0;
        result.push_back(point);
    }

    return result;
}

std::vector<geometry_msgs::msg::Point> ImageNav::screenToCamera(const std::vector<cv::Point2f>& cv_points, const cv::Mat& depth_img)
{
    const double f=733.33; // ピクセルサイズ[pixel] （2.2[mm] / 0.003[mm] = 733.33）
    const int cx=240;
    const int cy=150;

    double depth=0;
    
    std::vector<geometry_msgs::msg::Point> points;
    geometry_msgs::msg::Point point;
    for(const cv::Point& cv_point : cv_points)
    {
        depth = depth_img.at<float>(cv_point.y,cv_point.x);

        if(depth <= 0 || depth >= 20)
            depth = 0;
        // カメラ座標系をロボット座標系に変換
        point.y = -1.0*depth*(cv_point.x - cx)/f;
        point.z = depth*(cv_point.y - cy)/f;
        point.x = depth;
        points.push_back(point);
    }

    return points;
}

std::vector<geometry_msgs::msg::Point> ImageNav::generatePathPoints(const std::vector<geometry_msgs::msg::Point>& center_points, const std::vector<geometry_msgs::msg::Point>& obstacle_points)
{
    std::vector<geometry_msgs::msg::Point> path_points;
    constexpr double SAFE_DISTANCE = 1.5; // 障害物との安全距離

    for (const geometry_msgs::msg::Point& cp : center_points)
    {
        geometry_msgs::msg::Point adjusted_point = cp;

        for (const geometry_msgs::msg::Point& obs : obstacle_points)
        {
            double dx = cp.x - obs.x;
            double dy = cp.y - obs.y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < SAFE_DISTANCE)
            {
                double scale = SAFE_DISTANCE / dist;
                adjusted_point.x = obs.x + dx * scale;
                adjusted_point.y = obs.y + dy * scale;
                break;
            }
        }
        path_points.push_back(adjusted_point);
    }

    return path_points;
}

void ImageNav::publishPoints(const std::vector<geometry_msgs::msg::Point>& points, const std::string& frame_id, float r, float g, float b)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->now();
    marker.ns = "points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.1;  // 点のサイズ
    marker.scale.y = 0.1;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    for (const auto& point : points) {
        marker.points.push_back(point);
    }

    marker_pub_->publish(marker);
}


nav_msgs::msg::Path ImageNav::generatePath(const cv::Mat& cv_img, const cv::Mat& cv_depth_img)
{
    // 障害物（赤）のスクリーン座標を取得
    std::vector<cv::Point> obstacle_points = obstacle.detectObstacle(cv_img, cv_depth_img);

    cv::Mat detect_img = line.detectLine(cv_img);

    // 白線の開始位置（x座標）を検出
    std::vector<int> estimate_x = line.estimateLinePosition(detect_img);

    // スライドウィンドウ法で窓の中心点を抽出
    std::vector<cv::Point> left_points = line.SlideWindowMethod(detect_img, estimate_x[0], 0);
    std::vector<cv::Point> right_points = line.SlideWindowMethod(detect_img, estimate_x[1], 1);
    std::vector<cv::Point2f> center_points;

    for(size_t i=0; i < left_points.size(); ++i)
    {
        int x = (left_points[i].x + right_points[i].x) / 2;
        int y = (left_points[i].y + right_points[i].y) / 2;

        center_points.push_back(cv::Point(x, y));
    }
    center_points = line.BEVtoScreen(center_points);

    std::vector<cv::Point2f> obstacle_points_2f(obstacle_points.begin(), obstacle_points.end());
    std::vector<geometry_msgs::msg::Point> obstacle_pos = screenToCamera(obstacle_points_2f, cv_depth_img);
    std::vector<geometry_msgs::msg::Point> center_pos = screenToCamera(center_points, cv_depth_img);

    std::vector<geometry_msgs::msg::Point> path_points = generatePathPoints(center_pos, obstacle_pos);

    std::vector<geometry_msgs::msg::Point> spline_pos = interpolateSpline(path_points, 30);

    if(visualize_flag_)
    {
        std::vector<cv::Point2f> left_points_2f(left_points.begin(), left_points.end());
        std::vector<cv::Point2f> right_points_2f(right_points.begin(), right_points.end());
        std::vector<cv::Point2f> vis_left_points = line.BEVtoScreen(left_points_2f);
        std::vector<cv::Point2f> vis_right_points = line.BEVtoScreen(right_points_2f);
        cv::Mat point_img = line.WindowVisualizar(cv_img, vis_left_points, 0, cv::Scalar(0,255,0));
        point_img = line.WindowVisualizar(point_img, vis_right_points, 1, cv::Scalar(0,255,0));
        point_img = line.PointVisualizar(point_img, center_points, cv::Scalar(0, 0, 255));

        cv::imshow("center_point img", point_img);
        cv::waitKey(1);

        cv::Mat bev_img = line.WindowVisualizar(detect_img, left_points_2f, 0, cv::Scalar(0, 255, 0));
        bev_img = line.WindowVisualizar(bev_img, right_points_2f, 0, cv::Scalar(0, 255, 0));
        bev_img = line.PointVisualizar(bev_img, center_points, cv::Scalar(0, 0, 255));

        cv::imshow("bev img", bev_img);
        cv::waitKey(1);

        publishPoints(obstacle_pos);

        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "camera";

        sensor_msgs::msg::Image::SharedPtr rosimg = cv_bridge::CvImage(header, "bgr8", point_img).toImageMsg();
        image_pub_->publish(*rosimg);
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";
    for (const geometry_msgs::msg::Point& coord : spline_pos) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = coord.x;
        pose.pose.position.y = coord.y;
        path_msg.poses.push_back(pose);
    }
    path_pub_->publish(path_msg);

    return path_msg;
}

void ImageNav::ImageNavigation(void)
{
    if(!autonomous_flag_ || cv_img->image.empty() || cv_depth_img->image.empty())   return;
    nav_msgs::msg::Path path = generatePath(cv_img->image, cv_depth_img->image);
    double dx = path.poses[5].pose.position.x;
    double dy = path.poses[5].pose.position.y;

    double angle = std::atan2(dy, dx);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_max_;
    cmd_vel.angular.z = pid.cycle(angle);

    cmd_pub_->publish(cmd_vel);
}

}  // namespace imagenav
