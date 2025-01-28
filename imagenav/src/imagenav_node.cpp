#include "imagenav/imagenav_node.hpp"

#include "cmath"

namespace imagenav{

ImageNav::ImageNav(const rclcpp::NodeOptions &options) : ImageNav("", options) {}

ImageNav::ImageNav(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("imagenav_node", name_space, options)
{
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&ImageNav::ImageCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&ImageNav::OdomCallback, this, std::placeholders::_1));
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/line_detection/image", 10);
    // obstacle_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/obstacle_pos", 10);
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&ImageNav::PotentialMethod, this));
}

void ImageNav::ImageCallback(const sensor_msgs::msg::Image::SharedPtr img)
{
    cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img, img->encoding);
    if(cv_img->image.empty())    return;

    // 障害物の座標取得
    // ポテンシャル関数計算に使う
    cv::Point obstacles = obstacle.detectPotition(cv_img->image);
    // 白線認識
    cv::Mat bev_image = obstacle.toBEV(cv_img->image);
    cv::Mat mask_img = line.detectLine(bev_image);

    cv::imshow("BEV image data", mask_img);
    cv::waitKey(1);
}

void ImageNav::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    self_pose_.x = msg->pose.pose.position.x;
    self_pose_.y = msg->pose.pose.position.y;
}

double ImageNav::AttractiveForce(const Position& robot, const Position& goal)
{
    // ポテンシャル法 引力計算
    double diff_x = robot.x - goal.x;
    double diff_y = robot.y - goal.y;
    double force = 1 / sqrt(diff_x*diff_x + diff_y*diff_y);

    return force;
}

double ImageNav::RepulsiveForce(const Position& robot, const std::vector<Position>& obstacles)
{
    double force = 0; //初期化
    // ポテンシャル法 斥力計算（障害物）
    for(const auto& obs : obstacles)
    {
        double diff_x = robot.x - obs.x;
        double diff_y = robot.y - obs.y;
        force += 1 / sqrt(diff_x*diff_x + diff_y*diff_y);
    }

    return force;
}

double ImageNav::CalculatePotential(Position& robot, double bias_x, double bias_y)
{
    robot.x += bias_x;
    robot.y += bias_y;

    Position goal;
    goal.x = 10;
    goal.y = 0;

    std::vector<Position> obstacles;

    double attractive_force = AttractiveForce(robot, goal);
    double repulsive_force = RepulsiveForce(robot, obstacles);
    double potential = repulsive_force + attractive_force; // 後でゲインつける

    return potential;
}

void ImageNav::PotentialMethod()
{
    Position robot;
    robot.x = self_pose_.x;
    robot.y = self_pose_.y;
    
    double grad_x = -(CalculatePotential(robot, delta, 0) - CalculatePotential(robot, 0, 0))/ delta;
    double grad_y = -(CalculatePotential(robot, 0, delta) - CalculatePotential(robot, 0, 0))/ delta;

    double v = sqrt(grad_x*grad_x + grad_y*grad_y);

    grad_x /= v;
    grad_y /= v; 

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear_vel;
    cmd_vel.angular.z = std::atan(grad_y/grad_x);
}



}  // namespace imagenav
