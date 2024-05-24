#include "gnss_navigation/gnss_path_follower.hpp"

namespace gnss_navigation
{

GNSSPathFollower::GNSSPathFollower()
    :Node("gnss_path_follower"),
    autonomous_flag_(false),
    nav_start_flag_(false),
    init_base_flag_(true)
{
    declareParameter();
    initCommunication();
}

GNSSPathFollower::~GNSSPathFollower() {}

// パラメータ宣言
void GNSSPathFollower::declareParameter()
{
    this->declare_parameter("loop_freq", 20);
    this->declare_parameter("lookahead_distance", 1.5);
    this->declare_parameter("max_linear_vel", 1.0);
    this->declare_parameter("max_angular_vel", 0.7);
}

// pub, sub, パラメータ設定
void GNSSPathFollower::initCommunication(void)
{
    vectornav_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/vectornav/pose", 10, std::bind(&GNSSPathFollower::vectornavCallback, this, std::placeholders::_1));
    auto callback = [this](const std_msgs::msg::Empty::SharedPtr msg) { this->navStartCallback(msg); };
    nav_start_subscriber_ = this->create_subscription<std_msgs::msg::Empty>("/nav_start", 10, callback);
    path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>("/origin_gnss_path", 10, std::bind(&GNSSPathFollower::pathCallback, this, std::placeholders::_1));
    autonomous_flag_subscriber_ = this->create_subscription<std_msgs::msg::Bool>( "/autonomous", 10, std::bind(&GNSSPathFollower::autonomousFlagCallback, this, std::placeholders::_1));
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/cmd_vel", 2); 
    current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10); 
    current_lookahead_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_lookahead", 10); 

    this->get_parameter("loop_freq", freq_);
    this->get_parameter("lookahead_distance", lookahead_distance_);
    this->get_parameter("max_linear_vel", max_linear_vel_);
    this->get_parameter("max_angular_vel", max_angular_vel_);
}

// /vectornav/poseのコールバック関数
void GNSSPathFollower::vectornavCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        auto [x, y] = convertECEFtoUTM(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        current_position_x_ = x;
        current_position_y_ = y;
        current_yaw_ = calculateYawFromQuaternion(msg->pose.pose.orientation);
        double current_yaw_deg = radian2deg(current_yaw_);
        RCLCPP_DEBUG(this->get_logger(), "pose_x:%f, pose_y:%f, pose_yaw:%f deg", current_position_x_, current_position_y_, current_yaw_deg);
        if (init_base_flag_ && !(x == 0.0) && !(y == 0.0)) {
            setBasePose();
        } else {
            publishCurrentPose();
        }
    }

// global_pathの受け取り
void GNSSPathFollower::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    path_ = msg->poses;
}

// gnss_navigationで走行開始するか管理
void GNSSPathFollower::navStartCallback(const std_msgs::msg::Empty::SharedPtr&) {
    nav_start_flag_ = true;
    RCLCPP_INFO(this->get_logger(), "自律走行開始");
}

// autonomous_flag_を更新
void GNSSPathFollower::autonomousFlagCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    autonomous_flag_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Autonomous flag updated to: %s", autonomous_flag_ ? "true" : "false");
}

// 初期姿勢を取得
void GNSSPathFollower::setBasePose()
{
    vectornav_base_x_ = current_position_x_;
    vectornav_base_y_ = current_position_y_;
    vectornav_base_yaw_ = current_yaw_;
    init_base_flag_ = false;
}

// 初めのpathの向きを計算
void GNSSPathFollower::calcPathDirection()
{
    double dx = path_[1].pose.position.x - path_[0].pose.position.x;
    double dy = path_[1].pose.position.y - path_[0].pose.position.y;
    path_direction_ = std::atan2(dy, dx);
}

// TO DO 現在の姿勢を可視化
void GNSSPathFollower::publishCurrentPose()
{
    calcPathDirection();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now(); 
    pose_msg.header.frame_id = "map"; 

    tf2::Quaternion initial_q, current_q, result_q;
    initial_q.setRPY(0, 0, path_direction_ - vectornav_base_yaw_);
    initial_q = initial_q.inverse();

    tf2::Vector3 current_position(current_position_x_ - vectornav_base_x_,
        current_position_y_ - vectornav_base_y_, 
        0.0
    );

    tf2::Vector3 corrected_position = tf2::quatRotate(initial_q, current_position);

    pose_msg.pose.position.x = corrected_position.x();  
    pose_msg.pose.position.y = corrected_position.y();  
    pose_msg.pose.position.z = corrected_position.z();  

    current_q.setRPY(0, 0, current_yaw_);  // Roll, Pitch, Yawからクォータニオンを計算
    result_q = initial_q * current_q;

    pose_msg.pose.orientation = tf2::toMsg(result_q);
    current_pose_pub_->publish(pose_msg);
}

// TO DO 現在のlookahead pointを可視化
void GNSSPathFollower::publishLookahead()
{
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now(); 
    pose_msg.header.frame_id = "map"; 

    pose_msg.pose.position.x = path_[lookahead_index_].pose.position.x - vectornav_base_x_;  
    pose_msg.pose.position.y = path_[lookahead_index_].pose.position.y - vectornav_base_y_;  
    pose_msg.pose.position.z = 0.0;  

    current_lookahead_pub_->publish(pose_msg);
}

// lookahead_distanceを探索する
void GNSSPathFollower::findLookaheadDistance()
{
    // ロボットの位置に最も近い経路上の点を探索
    double closest_distance = std::numeric_limits<double>::max();
    size_t target_index = 0;
    for (size_t i = 0; i < path_.size(); ++i) {
        double dx = path_[i].pose.position.x - current_position_x_;
        double dy = path_[i].pose.position.y - current_position_y_;
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance < closest_distance) {
            closest_distance = distance;
            target_index = i;
        }
    }

    // 経路上の点から進行方向にlookahead_distance以上になるまで探索
    lookahead_index_ = target_index;
    for (size_t i = target_index; i < path_.size() - 1; ++i) {
        double segment_length = std::hypot(
            path_[i + 1].pose.position.x - current_position_x_,
            path_[i + 1].pose.position.y - current_position_y_);
        if (segment_length >= lookahead_distance_) {
            lookahead_index_ = i + 1;
            break;
        }
    }
}

// ラジアンから弧度法へ変換
double GNSSPathFollower::radian2deg(double rad)
{
    double deg = rad * 180 / M_PI;
    return deg;
}

// pure pursuit風アルゴリズムの経路追従
void GNSSPathFollower::followPath()
{
    if (path_.empty()) return;
    if (!nav_start_flag_) return;

    findLookaheadDistance();
    
    // lookahead_pointに対するロボットの位置との差を計算
    auto& lookahead_pose = path_[lookahead_index_].pose.position;
    double dx = lookahead_pose.x - current_position_x_;
    double dy = lookahead_pose.y - current_position_y_;
    double distance_to_lookahead = std::sqrt(dx * dx + dy * dy);

    publishLookahead();

    // 目標点に対する角度を計算
    double target_angle = std::atan2(dy, dx);
    double angle_diff = target_angle - current_yaw_;
    angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));

    // 角速度生成
    double controlled_angular_speed = std::copysign(std::min(std::abs(angle_diff), max_angular_vel_), angle_diff);

    // 速度生成
    double controlled_linear_speed = std::min(max_linear_vel_, distance_to_lookahead); 
    if (lookahead_index_ >= path_.size() - 10)   controlled_linear_speed = std::min(max_linear_vel_, distance_to_lookahead * 0.6);

    double angle_diff_deg = radian2deg(angle_diff);
    
    RCLCPP_INFO(this->get_logger(), "Current distance to target: %f meters, Current angle to target: %f deg", distance_to_lookahead, angle_diff_deg);

    geometry_msgs::msg::Vector3 cmd_vel;
    cmd_vel.x = controlled_linear_speed;
    cmd_vel.z = controlled_angular_speed;

    if ((distance_to_lookahead < 0.1) && (lookahead_index_ >= path_.size() - 10)){
        cmd_vel.x = 0.0;
        cmd_vel.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "Reached goal");
    }

    if (autonomous_flag_ == true) cmd_pub_->publish(cmd_vel);
}

// クオータニオンからオイラーへ変換
double GNSSPathFollower::calculateYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat) 
{
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

// ECEF座標系からUTM座標系への変換
std::pair<double, double> GNSSPathFollower::convertECEFtoUTM(double x, double y, double z) 
{
    PJ_CONTEXT *C = proj_context_create();
    PJ *P = proj_create_crs_to_crs(C,
                                "EPSG:4978", // ECEF
                                "EPSG:32654", // UTMゾーン54N
                                NULL);
    if (P == NULL) {
    std::cerr << "PROJ transformation creation failed." << std::endl;
    }
    PJ_COORD a, b;

    a.xyz.x = x;
    a.xyz.y = y;
    a.xyz.z = z;

    b = proj_trans(P, PJ_FWD, a);

    // リソースの解放
    proj_destroy(P);
    proj_context_destroy(C);

    return {b.enu.e, b.enu.n};
}

void GNSSPathFollower::loop(void)
{
    followPath();
}

} // namespace gnss_navigation

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gnss_navigation::GNSSPathFollower>();
    rclcpp::Rate loop_rate(node->freq_);
    while (rclcpp::ok()) {
        node->loop();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}