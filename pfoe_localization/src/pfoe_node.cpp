#include "pfoe_localization/pfoe_node.hpp"

namespace pfoe_localization
{

PfoeNode::PfoeNode(const rclcpp::NodeOptions& options)
: PfoeNode("", options) {}

PfoeNode::PfoeNode(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("pfoe_node", name_space, options)
{
  pub_ = create_publisher<std_msgs::msg::String>("/planning/nav_cmd", 10);
  pub_vel_ = create_publisher<steered_drive_msg::msg::SteeredDrive>("cmd_vel", 10);
  bool_pub_ = create_publisher<std_msgs::msg::Bool>("/pfoe_driving", 10);

  sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "pfoe/features",
    rclcpp::SensorDataQoS(),
    std::bind(&PfoeNode::featureCallback, this, std::placeholders::_1));
    
  const std::string share_dir = 
      ament_index_cpp::get_package_share_directory("pfoe_localization");
  const std::filesystem::path data_dir =
      std::filesystem::path(share_dir) / "data";
  
  if(pf_.init(data_dir.string() ,5) != 0){
    RCLCPP_ERROR(get_logger(), "failed to init particle filter");
    return ;
  }

  // pf_.selftest(0);

}

void PfoeNode::featureCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
  if(msg->data.size() != FEAT_DIM){
    RCLCPP_WARN(get_logger(), "unexpected feature dim: %zu", msg->data.size());
    return ;
  }
  pf_.cycle(msg->data);

  const auto result = pf_.decision();
  std_msgs::msg::String out;
  out.data =  (result.command == 2) ? "left" : (result.command == 3) ? "right" : "straight";
  pub_ -> publish(out);

  std_msgs::msg::Bool en;
  en.data = result.pfoe_en;
  bool_pub_ -> publish(en);

  auto msg_vel = std::make_shared<steered_drive_msg::msg::SteeredDrive>();
  msg_vel->velocity = result.linear_vel;
  msg_vel->steering_angle = result.angular_vel;
  if (result.pfoe_en){
    pub_vel_->publish(*msg_vel);
  }

}
  
}