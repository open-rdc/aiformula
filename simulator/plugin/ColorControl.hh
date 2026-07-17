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
  ColorControl();
  ColorControl(const std::shared_ptr<const sdf::Element> &_sdf);

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
  const int time;
  const std::string ROBOT_MODEL_NAME;
  const std::string COLOR_ENTITY_NAME;
  const ignition::math::Vector3d TARGET_POSITION;
  const double DETECTION_RADIUS;
  ignition::math::Color color_value;
  std::string color_name;
  double r;
  double g;
  double b;
  std::string color;

  double distance = 2 * DETECTION_RADIUS;
  // bool target_reached = false;
  std::chrono::steady_clock::duration reach_time;
  bool timer_started = false;
  bool color_changed = false;

  
};
}  // namespace gazebo
}  // namespace ignition

#endif