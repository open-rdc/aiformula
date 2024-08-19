#include "gnssnav/test_follower_node.hpp"

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
        std::bind(&Follower::onTimer, this));
}

void Follower::vectornavCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    // auto [x, y] = convertECEFtoUTM(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    // current_position_x_ = x;
    // current_position_y_ = y;

    RCLCPP_INFO(this->get_logger(), "current_position_x: %f\ncurrent_position_y: %f", current_position_x_, current_position_y_);

    current_yaw_ = calculateYawFromQuaternion(msg->pose.pose.orientation);
    // double current_yaw_deg = radian2deg(current_yaw_);
}

// received path
void Follower::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    point_ = msg->poses;
    // RCLCPP_INFO(this->get_logger(), "received %zu pose", point_.size());
}

void Follower::onTimer(){
    // if(!autonomous())
    //     return;

    size_t closest_idx_ =
        findNearestIndex();

    geometry_msgs::msg::Twist cmd_vel;
    if(closest_idx_ == point_.size() - 3){
        // cmd_vel.linear.x = 0.0;
        // cmd_vel.angular.z = 0.0;
        // RCLCPP_INFO(this->get_logger(), "Goal to reach");
    }else{
        // cmd_vel.linear.x = 3.0;
        // cmd_vel.angular.z = 3.0;

        // cmd_pub_->publish(cmd_vel);
    }
}

int Follower::findNearestIndex(){
    int num = 1;
    return num;
}


// クオータニオンからオイラーへ変換
double Follower::calculateYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat){

    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

double Follower::radian2deg(double rad){
    double deg = rad * (180 / M_PI);
    return deg;
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

bool Follower::autonomous(){
    if(!autonomous_flag_)
        return false;
    if(point_.empty())
        return false;
    return true;
}

}  // namespace gnssnav

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}
