#ifndef PYLON_CONTROL_PLUGIN_HH_
#define PYLON_CONTROL_PLUGIN_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/transport/Node.hh>

#include <atomic>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

namespace ignition
{
namespace gazebo
{
class PylonControl : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  PylonControl() = default;

  ~PylonControl() override;

  void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm, EventManager &_eventMgr) override;

  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

private:
  void LoadPath();
  void FindClearancePositions(EntityComponentManager &_ecm);
  std::vector<size_t> SelectPathIndices();
  void SpawnPylons();
  void RemovePylons();
  void CreatePylon(const std::string &_name, const math::Vector2d &_position, double _yaw);
  void SettingPylon(const std::shared_ptr<std_srvs::srv::Empty::Request> _req,
                    std::shared_ptr<std_srvs::srv::Empty::Response> _res);
  void ClearPylon(const std::shared_ptr<std_srvs::srv::Empty::Request> _req,
                  std::shared_ptr<std_srvs::srv::Empty::Response> _res);
  void ShutdownRos();

  std::string PATH_FILE_NAME;
  std::string PANEL_MODEL_NAME;
  std::string ROBOT_MODEL_NAME;
  std::string PYLON_MODEL_NAME;
  std::string SETTING_SERVICE_NAME;
  std::string CLEAR_SERVICE_NAME;

  double clearance_{20.0};
  double group_interval_{10.0};
  double lateral_range_{3.0};
  double pylon_interval_{0.5};
  int group_num_{3};
  int min_pylon_num_{2};
  int max_pylon_num_{4};

  std::string model_file_path_;
  std::string create_service_;
  std::string remove_service_;

  std::vector<math::Vector2d> path_;
  std::vector<math::Vector2d> clearance_positions_;
  std::vector<std::string> pylon_names_;
  int pylon_count_{0};

  std::atomic<bool> setting_requested_{false};
  std::atomic<bool> clear_requested_{false};
  bool ros_ready_{false};

  std::mt19937 random_engine_{std::random_device{}()};
  transport::Node node_;

  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr setting_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_service_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};
}  // namespace gazebo
}  // namespace ignition

#endif
