#include "pfoe_localization/pfoe_node.hpp"

namespace pfoe_localization
{

PfoeNode::PfoeNode(const rclcpp::NodeOptions& options)
: PfoeNode("", options) {}

PfoeNode::PfoeNode(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("pfoe_node", name_space, options),
  nav_cmd_topic_(get_parameter("nav_cmd_topic").as_string()),
  cmd_vel_topic_(get_parameter("cmd_vel_topic").as_string()),
  feature_topic_(get_parameter("feature_topic").as_string()),
  pfoe_driving_topic_(get_parameter("pfoe_driving_topic").as_string()),
  pfoe_enabled_(get_parameter("pfoe_enabled").as_bool()),
  pfoe_direct_action_enabled_(get_parameter("pfoe_direct_action_enabled").as_bool()),
  prediction_range_(get_parameter("prediction_range").as_int())
{
  if (nav_cmd_topic_.empty() || cmd_vel_topic_.empty() || feature_topic_.empty() || pfoe_driving_topic_.empty()) {
    throw std::invalid_argument("pfoe_node topic parameters must not be empty");
  }
  const rclcpp::QoS qos(10);
  nav_cmd_publisher_ = create_publisher<std_msgs::msg::String>(nav_cmd_topic_, qos);
  cmd_vel_publisher_ = create_publisher<steered_drive_msg::msg::SteeredDrive>(cmd_vel_topic_, qos);
  pfoe_driving_publisher_ = create_publisher<std_msgs::msg::Bool>(pfoe_driving_topic_, qos);

  feature_subscription_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    feature_topic_,rclcpp::SensorDataQoS(),
    std::bind(&PfoeNode::featureCallback, this, std::placeholders::_1));
    
  const std::string share_dir = 
      ament_index_cpp::get_package_share_directory("pfoe_localization");
  const std::filesystem::path data_dir =
      std::filesystem::path(share_dir) / "data";
  
  if (pfoe_enabled_) {
    if (pf_.init(data_dir.string(), prediction_range_) != 0) {
      throw std::runtime_error("failed to init particle filter");
    }
  }

}

void PfoeNode::featureCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
  if(!pfoe_enabled_){
    return ;
  }

  if(msg->data.size() != FEAT_DIM){
    RCLCPP_WARN(get_logger(), "unexpected feature dim: %zu", msg->data.size());
    return ;
  }
  pf_.cycle(msg->data);
  const auto result = pf_.decision();

  std_msgs::msg::String out;
  out.data =  (result.command == 2) ? "left" : (result.command == 3) ? "right" : "straight";
  nav_cmd_publisher_->publish(out);
  
  if(pfoe_direct_action_enabled_){
    std_msgs::msg::Bool en;
    en.data = result.pfoe_en;
    pfoe_driving_publisher_->publish(en);
    auto msg_vel = std::make_shared<steered_drive_msg::msg::SteeredDrive>();
    msg_vel->velocity = result.linear_vel;
    msg_vel->steering_angle = result.angular_vel;
    if(result.pfoe_en){
      cmd_vel_publisher_->publish(*msg_vel);
    }
  }

}
  
}