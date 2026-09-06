#ifndef COLOR_CONTROL_PLUGIN_HH_
#define COLOR_CONTROL_PLUGIN_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Material.hh>
#include <ignition/math/Color.hh>

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

namespace ignition
{
namespace gazebo
{
class ColorControl : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  // Constructor
  ColorControl() = default;

  // Destructor
  ~ColorControl() override;

  // ISystemConfigure method
  void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm, EventManager &_eventMgr) override;

  // ISystemPreUpdate method
  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

private:
  void FindColorEntities(EntityComponentManager &_ecm);
  void ToggleSignal(const std::shared_ptr<std_srvs::srv::Empty::Request> _req,
                    std::shared_ptr<std_srvs::srv::Empty::Response> _res);
  void ShutdownRos();

  std::vector<Entity> ColorEntities;
  std::string COLOR_ENTITY_NAME;
  std::string SERVICE_NAME;

  std::atomic<bool> is_green_{false};
  bool ros_ready_{false};

  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};
}  // namespace gazebo
}  // namespace ignition

#endif
