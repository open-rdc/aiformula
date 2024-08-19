#include "gnssnav/follower_node.hpp"

namespace gnssnav{

Follower::Follower(const rclcpp::NodeOptions& options) : Follower("", options) {}

// pub, sub, param
Follower::Follower(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("gnssnav_follower_node", name_space, options)
{
    auto callback = [this](const std_msgs::msg::Empty::SharedPtr msg) { this->navStartCallback(msg); };

    autonomous_flag_subscriber_ = this->create_subscription<std_msgs::msg::Bool>("/autonomous", 10, std::bind(&Follower::autonomousFlagCallback, this, std::placeholders::_1));
    nav_start_subscriber_ = this->create_subscription<std_msgs::msg::Empty>("/nav_start", 10, callback);
    vectornav_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/vectornav/pose", 10, std::bind(&Follower::vectornavCallback, this, std::placeholders::_1));
    path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>("/gnss_path", 10, std::bind(&Follower::pathCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);
    current_ld_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_ld", 10); 
    theta_pub_ =this->create_publisher<std_msgs::msg::Float64>("/theta", 10);

    this->get_parameter("lookahead_gain", ld_gain_);
    this->get_parameter("cte_gain", cte_gain_);
    this->get_parameter("min_lookahead_distance", ld_min_);
    this->get_parameter("max_linear_vel", v_max_);
    this->get_parameter("max_angular_vel", w_max_);
    this->get_parameter("follower_freq", freq);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(freq),
        std::bind(&Follower::loop, this));
}

// vectornav/pose callback
void Follower::vectornavCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    auto [x, y] = convertECEFtoUTM(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    current_position_x_ = x;
    current_position_y_ = y;
    //if(!init_base_flag_ && count >= 100){
    //	 init_yaw = calculateYawFromQuaternion(msg->pose.pose.orientation);
    //}
    // RCLCPP_INFO(this->get_logger(), "current_position_x: %f\ncurrent_position_y: %f", current_position_x_, current_position_y_);

    current_yaw_ = calculateYawFromQuaternion(msg->pose.pose.orientation);
    double current_yaw_deg = radian2deg(current_yaw_);
    // RCLCPP_INFO(this->get_logger(), "current_yaw_deg: %f", current_yaw_deg);

    // RCLCPP_INFO(this->get_logger(), "base_x: %f\ncurrent_position_x: %f\nbase_yaw: %f", vectornav_base_x_, current_position_x_, vectornav_base_yaw_);
    // std::cerr << init_base_flag_ << std::endl;
    if(!init_base_flag_) {
        setBasePose();
    } else {
        publishCurrentPose();
    }
}

// received path
void Follower::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    point_ = msg->poses;
    // RCLCPP_INFO(this->get_logger(), "received %zu pose", point_.size());
}

// gnssnav permit
void Follower::navStartCallback(const std_msgs::msg::Empty::SharedPtr&) {
    nav_start_flag_ = true;
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

    // for(const auto &pose : point_){
           // RCLCPP_INFO(this->get_logger(), "path_x:%f , path_y:%f", pose.pose.position.x, pose.pose.position.y);
   // }
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
    for(idx_ = 0;idx_ < point_.size(); idx_++){
        double dx = point_[idx_].pose.position.x - front_wheel_pos.position.x;
        double dy = point_[idx_].pose.position.y - front_wheel_pos.position.y;
        distance_ = std::hypot(dx, dy);

 //std::cerr << "idx:" << idx_ << std::endl;
    // std::cerr << "point_x:" << point_[idx_].pose.position.x << "point_y:"<< point_[idx_].pose.position.y << std::endl;
	//std::cerr << "wheel_pos_y:" << front_wheel_pos.position.y << std::endl;
//std::cerr << "dx:" << dx << "  " <<"dy" << dy << std::endl;
//if(idx_ >= 100)
//std::cerr << "distance" << distance_ << std::endl;

        if(distance_ > ld_ && idx_ > pre_point_idx){
		    pre_point_idx = idx_;
            RCLCPP_INFO(this->get_logger(), "idx:%d", idx_);
            break;
        }
    }
    return ;
}

// 目標地点を探索する
void Follower::findLookaheadDistance(){
    wheel_base_ = 600;
    double front_x_ =
	    current_position_x_;
        //current_position_x_ + wheel_base_ / 2.0 * std::cos(pose_orientation_z_);
    double front_y_ =
	    current_position_y_;
        //current_position_y_ + wheel_base_ / 2.0 * std::sin(pose_orientation_z_);

    geometry_msgs::msg::Pose front_wheel_pos;
    front_wheel_pos.position.x = front_x_;
    front_wheel_pos.position.y = front_y_;

    //std::cerr << "current_y_2: "<<current_position_y_<< std::endl;
    // 前の車輪から一番近い経路点を見つける
    findNearestIndex(front_wheel_pos);

    if(idx_ > 0){
        pre_point_idx = idx_ - 1;
    }else{
        pre_point_idx = idx_;
    }
}

double Follower::radian2deg(double rad){
    double deg = rad * (180 / M_PI);
    return deg;
}

// not scope 経路に対しての横方向のズレを計算
double Follower::calculateCrossError(){
    double dx = point_[idx_].pose.position.x - current_position_x_;
    double dy = point_[idx_].pose.position.y - current_position_y_;

    double target_angle = M_PI -  std::atan2(dy, dx);
	double target_angle_deg = radian2deg(target_angle);
    // RCLCPP_INFO(this->get_logger(), "target_angle_deg: %f", target_angle_deg);
    theta = target_angle - current_yaw_;
    theta = std::atan2(std::sin(theta), std::cos(theta));

   // RCLCPP_INFO(this->get_logger(), "target_angle:%f, theta_deg: %f", target_angle, radian2deg(theta));

    double cross_error = dy * std::cos(theta) - dx * std::sin(theta);

    return cross_error;
}

// pointとpre_pointを結ぶ先に対する現在の車体の角度を計算
double Follower::calculateHeadingError(){
    double traj_theta = 
        std::atan2(point_[idx_].pose.position.y - point_[pre_point_idx].pose.position.y,
                   point_[idx_].pose.position.x - point_[pre_point_idx].pose.position.x);

    double heading_error = traj_theta - theta;
    heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));

    return heading_error;
}

void Follower::followPath(){
    if(point_.empty()){
        std::cerr << "point_empty error" << std::endl;
        return;
    }
    //if(!autonomous_flag_){
        // std::cerr << "nav_start_flag error" << std::endl;
     //   return;
    //}

    ld_ = ld_gain_ * v_ + ld_min_; 

    findLookaheadDistance();
    publishLookahead();

    // 横方向のズレ
    double cte = calculateCrossError();
    // pointとpre_pointを結ぶ先に対する現在の車体の角度
    double he = calculateHeadingError();

    // v_ = std::min(v_max_, ld_);
    v_ = 1.5;
    w_ = he + std::atan2(cte_gain_ * cte, v_);

    double theta_deg = radian2deg(theta);
    RCLCPP_INFO(this->get_logger(), "distance: %f meters, theta_deg: %f deg, idx %d", distance_, theta_deg, idx_);

    //theta_degをパブリッシュ
    theta_pub_->publish(theta_deg);
    //RCLCPP_INFO(this->get_logger(), "hand error : %f", he);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = v_;
    cmd_vel.angular.z = std::max(std::min(theta, 1.0), -1.0);

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
    PJ_CONTEXT *C = proj_context_create();
    PJ *P = proj_create_crs_to_crs(C,
                                "EPSG:4978", // ECEF
                                "EPSG:32654", // UTMゾーン54N
                                NULL);
    if(P == NULL) {
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
