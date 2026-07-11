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
// クラス定義内で ISystemPreUpdate を使用するためにクラスを宣言
class LightControl : public System, public ISystemPreUpdate
{
public:
  // Constructor
  LightControl() = default;

  // ISystemPreUpdate method
  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

private:
  // Function to find the light entity by name
  void FindLightEntities(EntityComponentManager &_ecm);
  void FindModelEntities(EntityComponentManager &_ecm);
  void SetGreen();
  void SetRed();
  void TimerReset();

  std::vector<Entity> LightEntities;
  Entity RobotEntity = kNullEntity;

  const std::string ROBOT_MODEL_NAME = "ai_car1";
  const ignition::math::Vector3d TARGET_POSITION{80.0, 5.0, 0.0};
  const double DETECTION_RADIUS = 10.0;
  double distance = 20.0;

  std::chrono::steady_clock::duration reach_time;
  bool timer_started = false;
  bool color_changed = false;

  double r = 1.0;
  double g = 0.0;
  const double b = 0.0;
  std::string color = "Red";
  const std::string LIGHT_ENTITY_NAME = "led";
};
}  // namespace gazebo
}  // namespace ignition

#endif