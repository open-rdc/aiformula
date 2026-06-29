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

  // Time accumulator for color cycling
  double time = 0.0;

  std::vector<Entity> lightEntites; // list of light entities
};
}  // namespace gazebo
}  // namespace ignition

#endif