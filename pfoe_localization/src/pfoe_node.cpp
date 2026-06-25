#include "pfoe_localization/pfoe_node.hpp"

namespace pfoe_localization
{

PfoeNode::PfoeNode(const rclcpp::NodeOptions& options)
: PfoeNode("", options) {}

PfoeNode::PfoeNode(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("pfoe_node", name_space, options)
{
  pub_ = create_publisher<std_msgs::msg::String>("/planning/nav_cmd", 10);
  debug_pub_ = create_publisher<std_msgs::msg::Int32>("/pfoe_cmd", 10);

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

  const int cmd = static_cast<int>(pf_.decision());
  std_msgs::msg::String out;
  out.data =  (cmd == 2) ? "left" : (cmd == 3) ? "right" : "straight";
  pub_ -> publish(out);

  std_msgs::msg::Int32 dbg;
  dbg.data = cmd;
  debug_pub_->publish(dbg);

}
  
}