#include "gnssnav/follower_node.hpp"

#include "utilities/utils.hpp"
#include <cmath>

using namespace utils;

namespace gnssnav{

Follower::Follower(const rclcpp::NodeOptions& options) : Follower("", options) {}

// pub, sub, param
Follower::Follower(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("gnssnav_follower_node", name_space, options),
is_debug(get_parameter("debug_flag").as_bool()),
freq_ms(get_parameter("interval_ms").as_int()),
pid(get_parameter("interval_ms").as_int()),
laps(get_parameter("laps").as_int()),
ld_gain_(get_parameter("lookahead_gain").as_double()),
ld_min_(get_parameter("min_lookahead_distance").as_double()),
v_max_(get_parameter("max_linear_vel").as_double()),
w_max_(get_parameter("max_angular_vel").as_double()),
wheel_base_(get_parameter("wheelbase").as_double())
{
    auto callback = [this](const std_msgs::msg::Empty::SharedPtr msg) { this->navStartCallback(msg); };

    autonomous_flag_subscriber_ = this->create_subscription<std_msgs::msg::Bool>("/autonomous", 10, std::bind(&Follower::autonomousFlagCallback, this, std::placeholders::_1));
    nav_start_subscriber_ = this->create_subscription<std_msgs::msg::Empty>("/nav_start", 10, callback);
    vectornav_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/vectornav/pose", 10, std::bind(&Follower::vectornavCallback, this, std::placeholders::_1));
    path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>("/origin_gnss_path", 10, std::bind(&Follower::pathCallback, this, std::placeholders::_1));
    restart_subscriber_ = this->create_subscription<std_msgs::msg::Empty>("/restart", 10, std::bind(&Follower::restartCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);
    current_ld_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_ld", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(freq_ms),
        std::bind(&Follower::followPath, this));

    C = proj_context_create();
    P = proj_create_crs_to_crs(C,
        "EPSG:4978", // ECEF
        "EPSG:32654", // UTMゾーン54N
        NULL);

    pid.gain(get_parameter("p_gain").as_double(), get_parameter("i_gain").as_double(), get_parameter("d_gain").as_double());
}

Follower::~Follower(){
    // リソースの解放
    proj_destroy(P);
    proj_context_destroy(C);
}

// vectornav/pose callback
void Follower::vectornavCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if(point_.empty())
        return;

    auto [x, y] = convertECEFtoUTM(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    current_position_x_ = x;
    current_position_y_ = y;

    current_yaw_ = calculateYawFromQuaternion(msg->pose.pose.orientation) + (M_PI/2.0);
    // RCLCPP_INFO(this->get_logger(), "current yaw:%lf°", rtod(current_yaw_));

    if(!init_base_flag_) {
        setBasePose();
    } else {
        publishCurrentPose();
    }
}

// gnssnav permit
void Follower::navStartCallback(const std_msgs::msg::Empty::SharedPtr&) {
    idx_ = 0;
    init_d = false;
    RCLCPP_ERROR(this->get_logger(), "idx_がリセットされます");
}

// autonomous_flag_を更新
void Follower::autonomousFlagCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    autonomous_flag_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Autonomous flag updated to: %s", autonomous_flag_ ? "true" : "false");
}

// restart
void Follower::restartCallback(const std_msgs::msg::Empty::SharedPtr msg){
    pid.reset();
    RCLCPP_INFO(this->get_logger(), "再起動");
}

void Follower::setBasePose(){
    vectornav_base_x_ = current_position_x_;
    vectornav_base_y_ = current_position_y_;
    vectornav_base_yaw_ = current_yaw_;

    std::cerr << "set Base Pose" << std::endl;
    init_base_flag_ = true;

     for(const auto &pose : point_){
            RCLCPP_INFO_EXPRESSION(this->get_logger(), is_debug, "path_x:%f , path_y:%f", pose.pose.position.x, pose.pose.position.y);
    }
}

// 現在地をパブリッシュ
void Follower::publishCurrentPose(){
    double dx = point_[1].pose.position.x - point_[0].pose.position.x;
    double dy = point_[1].pose.position.y - point_[0].pose.position.y;
    double path_direction_ = std::atan2(dy, dx);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "map";

    tf2::Quaternion initial_q, current_q, result_q;
    initial_q.setRPY(0, 0, path_direction_ - vectornav_base_yaw_);
    initial_q = initial_q.inverse();

    tf2::Vector3 current_position(
        current_position_x_ - vectornav_base_x_,
        current_position_y_ - vectornav_base_y_,
        0.0
        );

    tf2::Vector3 corrected_position = tf2::quatRotate(initial_q, current_position);

    pose_msg.pose.position.x = corrected_position.x();
    pose_msg.pose.position.y = corrected_position.y();
    pose_msg.pose.position.z = corrected_position.z();

    current_q.setRPY(0, 0, current_yaw_);
    result_q = initial_q * current_q;

    pose_msg.pose.orientation = tf2::toMsg(result_q);
    current_pose_pub_->publish(pose_msg);
}

void Follower::publishLookahead(){
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "map";

    pose_msg.pose.position.x = point_[idx_].pose.position.x - vectornav_base_x_;
    pose_msg.pose.position.y = point_[idx_].pose.position.y - vectornav_base_y_;
    pose_msg.pose.position.z = 0.0;

    current_ld_pub_->publish(pose_msg);
}

// 目標地点を探索する
double Follower::findLookaheadDistance(){
    double ld_ = ld_gain_ * v_ + ld_min_;

    for(idx_ = pre_point_idx;idx_ < point_.size(); idx_++){
        double dx = point_[idx_].pose.position.x - current_position_x_;
        double dy = point_[idx_].pose.position.y - current_position_y_;
        double distance_ = std::hypot(dx, dy);

        if(distance_ > ld_ && idx_ > pre_point_idx && point_[idx_].pose.position.x > 30000){
		    pre_point_idx = idx_ - 1;
            return distance_;
        }
    }
}

// double Follower::ObstaclePotential()
// {
//     Force force;

//     double dx = point_[idx_].pose.position.x - obstacle.x;
//     double dy = point_[idx_].pose.position.y - obstacle.y;

//     obstacle.distance = std::hypot(dx, dy);
//     obstacle.theta = std::atan2(dy, dx);
//         // k, ld_ ,th のパラメータ設定
//         // distanceはlookaheadのdistance
//         // obstacleのthetaとdistanceがほしい lookaheadとの角度がtheta
//     // double force_r.x = -k_r*(ld_ - distance_)*std::cos(theta);
//     // double force_r.y = -k_r*(ld_ - distance_)*std::sin(theta);

//     force.x = -k_o*(th - obstacle.distance)*std::cos(obstacle.theta);
//     force.y = -k_o*(th - obstacle.distance)*std::sin(obstacle.theta);

//     dx = point_[idx_].pose.position.x - current_position_x_ + force.x;
//     dy = point_[idx_].pose.position.y - current_position_y_ + force.y;

//     double target_angle = std::atan2(dy, dx);

//     double angle = current_yaw_;
//     angle = std::atan2(std::sin(angle), std::cos(angle));

//     double theta = target_angle - angle;
//     theta = std::atan2(std::sin(theta), std::cos(theta));

//     return theta;
// }

double Follower::calculateCrossError(){
    double dx = point_[idx_].pose.position.x - current_position_x_;
    double dy = point_[idx_].pose.position.y - current_position_y_;

    double target_angle = std::atan2(dy, dx);

    double angle = current_yaw_;
    angle = std::atan2(std::sin(angle), std::cos(angle));

    double theta = target_angle - angle;
    theta = std::atan2(std::sin(theta), std::cos(theta));

    RCLCPP_INFO_EXPRESSION(this->get_logger(), is_debug, "target:%lf° current:%lf°", rtod(target_angle), rtod(angle));
    return theta;
}

void Follower::followPath(){
    if(point_.empty() || !autonomous_flag_) return;

    double distance = findLookaheadDistance();
    publishLookahead();

    // 目標地点との角度のズレ
    // double theta = calculateCrossError();
    double theta = calculateCrossError();

    v_ = v_max_;
    w_ = pid.cycle(theta);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = constrain(v_, -v_max_, v_max_);
    cmd_vel.angular.z = constrain(w_, -w_max_, w_max_);

    // 完走した判定
    if(idx_ >= point_.size()){
        if(laps == 0){
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Goal to reach");
        }else{
            idx_ = 0;
            pre_point_idx = 0;
            laps--;
            RCLCPP_INFO(this->get_logger(), "laps : %d", laps);
        }
    }

    cmd_pub_->publish(cmd_vel);
}

// クオータニオンからオイラーへ変換
double Follower::calculateYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat){

    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

std::pair<double, double> Follower::convertECEFtoUTM(double x, double y, double z){
    if(P == NULL) {
        std::cerr << "PROJ transformation creation failed." << std::endl;
    }
    PJ_COORD a, b;

    a.xyz.x = x;
    a.xyz.y = y;
    a.xyz.z = z;

    b = proj_trans(P, PJ_FWD, a);

    return {b.enu.e, b.enu.n};
}


}  // namespace gnssnav
