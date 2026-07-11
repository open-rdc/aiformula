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
class ColorControl : public System, public ISystemPreUpdate
{
public:
  // Constructor
  ColorControl() = default;

  // ISystemPreUpdate method
  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

private:
  // Function to find the color entity by name
  void FindColorEntities(EntityComponentManager &_ecm);
  void FindModelEntities(EntityComponentManager &_ecm);
  void SetGreen();
  void SetRed();
  void TimerReset();

  std::vector<Entity> ColorEntities;
  Entity RobotEntity = kNullEntity;
  
  const std::string ROBOT_MODEL_NAME = "ai_car1";
  const ignition::math::Vector3d TARGET_POSITION{80.0, 5.0, 0.0};
  const double DETECTION_RADIUS = 10.0;
  double distance = 20.0;

  // bool target_reached = false;
  std::chrono::steady_clock::duration reach_time;
  bool timer_started = false;
  bool color_changed = false;

  const std::string COLOR_ENTITY_NAME = "screen_visual";
  double r = 1.0;
  double g = 0.0;
  const double b = 0.0;
  std::string color = "Red";


};
}  // namespace gazebo
}  // namespace ignition

#endif