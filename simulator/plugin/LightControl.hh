#ifndef LIGHT_CONTROL_PLUGIN_HH_
#define LIGHT_CONTROL_PLUGIN_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Light.hh>
#include <ignition/math/Color.hh>
#include <ignition/gazebo/Light.hh>

#include <vector>

namespace ignition
{
namespace gazebo
{
class LightControl : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  // Constructor
  LightControl() = default;

  // ISystemConfigure method
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &_eventMgr) override;

  // ISystemPreUpdate method
  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

private:
  void FindLightEntities(EntityComponentManager &_ecm);
  void FindModelEntities(EntityComponentManager &_ecm);
  void SetGreen();
  void SetRed();
  void TimerReset();

  std::vector<Entity> LightEntities;
  Entity RobotEntity = kNullEntity;

  int time;
  std::string ROBOT_MODEL_NAME;
  std::string LIGHT_ENTITY_NAME;
  ignition::math::Vector3d TARGET_POSITION;
  double DETECTION_REACTION;
  ignition::math::Color color_value;
  std::string color_name;
  double r;
  double g;
  double b;
  double x;
  double y;

  std::chrono::steady_clock::duration reach_time;
  bool timer_started = false;
  bool color_changed = false;

};
}  // namespace gazebo
}  // namespace ignition

#endif