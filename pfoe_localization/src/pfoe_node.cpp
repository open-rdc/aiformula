#include "pfoe_localization/pfoe_node.hpp"

namespace pfoe_localization
{

PfoeNode::PfoeNode(const rclcpp::NodeOptions& options)
: PfoeNode("", options) {}

PfoeNode::PfoeNode(const std::string& name_space, const rclcpp::NodeOptions& options)
: rclcpp::Node("pfoe_node", name_space, options)
{
  pub_ = create_publisher<std_msgs::msg::Int32>("pfoe/command", 10);

  sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "pfoe/features",
    rclcpp::SensorDataQoS(),
    std::bind(&PfoeNode::featureCallback, this, std::placeholders::_1));
    
  const std::string share_dir = 
      ament_index_cpp::get_package_share_directory("e2e_planner");
  const std::filesystem::path data_dir =
      std::filesystem::path(share_dir) / "data";
  
  if(pf_.init(data_dir.string() ,5) != 0){
    RCLCPP_ERROR(get_logger(), "failed to init particle filter");
    return ;
  }

}

void PfoeNode::featureCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
  if(msg->data.size() != FEAT_DIM){
    RCLCPP_WARN(get_logger(), "unexpected feature dim: %zu", msg->data.size());
    return ;
  }
  pf_.cycle(msg->data);

  float cmd = pf_.decision();
  std_msgs::msg::Int32 out;
  out.data = static_cast<int>(cmd);
  pub_ -> publish(out);
}
  
}

int main(){
  
}