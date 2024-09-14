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
freq(get_parameter("interval_ms").as_int()),
ld_gain_(get_parameter("lookahead_gain").as_double()),
p_gain_(get_parameter("p_gain").as_double()),
i_gain_(get_parameter("i_gain").as_double()),
d_gain_(get_parameter("d_gain").as_double()),
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

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);
    current_ld_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_ld", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(freq),
        std::bind(&Follower::loop, this));

    C = proj_context_create();
    P = proj_create_crs_to_crs(C,
        "EPSG:4978", // ECEF
        "EPSG:32654", // UTMゾーン54N
        NULL);
}

Follower::~Follower(){
    // リソースの解放
    proj_destroy(P);
    proj_context_destroy(C);
}

// received path
void Follower::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    point_ = msg->poses;
    path_get_flag_ = true;
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
    pre_point_idx = 0;
    RCLCPP_ERROR(this->get_logger(), "idx_がリセットされます");
    RCLCPP_INFO(this->get_logger(), "自律走行開始");
}

// autonomous_flag_を更新
void Follower::autonomousFlagCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    autonomous_flag_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Autonomous flag updated to: %s", autonomous_flag_ ? "true" : "false");
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
    path_direction_ = std::atan2(dy, dx);

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

    pose_orientation_z_ = pose_msg.pose.orientation.z;
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

// 最も近い経路点を見つける(座標とidx)
void Follower::findNearestIndex(geometry_msgs::msg::Pose front_wheel_pos){
    for(idx_ = pre_point_idx;idx_ < point_.size(); idx_++){
        double dx = point_[idx_].pose.position.x - front_wheel_pos.position.x;
        double dy = point_[idx_].pose.position.y - front_wheel_pos.position.y;
        distance_ = std::hypot(dx, dy);

        if(distance_ > ld_ && idx_ > pre_point_idx && point_[idx_].pose.position.x > 30000){
		    pre_point_idx = idx_ - 1;
            break;
        }
    }
    return ;
}

// 目標地点を探索する
void Follower::findLookaheadDistance(){
    double front_x_ =
        current_position_x_ + wheel_base_ / 2.0 * std::cos(pose_orientation_z_);
    double front_y_ =
        current_position_y_ + wheel_base_ / 2.0 * std::sin(pose_orientation_z_);

    geometry_msgs::msg::Pose front_wheel_pos;
    front_wheel_pos.position.x = front_x_;
    front_wheel_pos.position.y = front_y_;

    // 前の車輪から一番近い経路点を見つける
    findNearestIndex(front_wheel_pos);
}

// not scope 経路に対しての横方向のズレを計算
double Follower::calculateCrossError(){
    double dx = point_[idx_].pose.position.x - current_position_x_;
    double dy = point_[idx_].pose.position.y - current_position_y_;

    double target_angle = std::atan2(dy, dx);

    double angle = current_yaw_;
    angle = std::atan2(std::sin(angle), std::cos(angle));

    theta = target_angle - angle;
    theta = std::atan2(std::sin(theta), std::cos(theta));

    double cross_error = dy * std::cos(current_yaw_) - dx * std::sin(current_yaw_);

    RCLCPP_INFO_EXPRESSION(this->get_logger(), is_debug, "target:%lf° current:%lf°", rtod(target_angle), rtod(angle));
    return cross_error;
}

// pointとpre_pointを結ぶ先に対する現在の車体の角度を計算
double Follower::calculateHeadingError(){
    double traj_theta =
        std::atan2(point_[idx_].pose.position.y - point_[pre_point_idx].pose.position.y,
                   point_[idx_].pose.position.x - point_[pre_point_idx].pose.position.x);

    double heading_error = traj_theta - theta;
    heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error)) *-1;

    return heading_error;
}

void Follower::followPath(){
    if(point_.empty()){
        std::cerr << "point_empty error" << std::endl;
        return;
    }

    ld_ = ld_gain_ * v_ + ld_min_;

    findLookaheadDistance();
    publishLookahead();

    // 横方向のズレ
    double cte = calculateCrossError();
    if(!init_d){
        init_d = true;
        cte_error = cte;
        prev_cte = cte;
        cte_sum = cte;
    }else{
        cte_error = cte - prev_cte;
        cte_sum += cte;
        prev_cte = cte;
    }
    // pointとpre_pointを結ぶ先に対する現在の車体の角度
    // double he = calculateHeadingError();

    v_ = v_max_;
    // RCLCPP_INFO(this->get_logger(), "distance : %lf idx_ : %d", distance_, idx_);
    w_ = p_gain_*cte + i_gain_*cte_sum + d_gain_*cte_error;
    
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = v_;
    cmd_vel.angular.z = constrain(w_, -w_max_, w_max_);

    // 完走した判定
    if(idx_ >= point_.size() - 5){
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "Goal to reach");
    }

    //  自律フラグonのときのみパブリッシュ
    if(autonomous_flag_){
        // std::cerr << "nav_start_flag error" << std::endl;
        cmd_pub_->publish(cmd_vel);
    }
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

void Follower::loop(void){
    followPath();
}

}  // namespace gnssnav

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gnssnav::Follower>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
