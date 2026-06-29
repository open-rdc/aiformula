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
  Entity modelEntity{kNullEntity};
  // Function to find the color entity by name
  void FindColorEntities(EntityComponentManager &_ecm);

  // Time accumulator for color cycling
  double time = 0.0;

  std::vector<Entity> ColorEntities;
};
}  // namespace gazebo
}  // namespace ignition

#endif