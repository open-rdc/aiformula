#ifndef COLOR_CONTROL_PLUGIN_HH_
#define COLOR_CONTROL_PLUGIN_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Material.hh>
#include <ignition/math/Color.hh>

#include <vector>

namespace ignition
{
namespace gazebo
{
class ColorControl : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  // Constructor
  ColorControl() = default;

  // ISystemConfigure method
  void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm, EventManager &_eventMgr) override;

  // ISystemPreUpdate method
  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

private:
  void FindColorEntities(EntityComponentManager &_ecm);
  void FindModelEntities(EntityComponentManager &_ecm);
  void SetGreen();
  void SetRed();
  void TimerReset();

  std::vector<Entity> ColorEntities;
  Entity RobotEntity = kNullEntity;
  
  int time;
  std::string ROBOT_MODEL_NAME;
  std::string COLOR_ENTITY_NAME;
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