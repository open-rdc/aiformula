#include "LightControl.hh"

#include <ignition/plugin/Register.hh>

#include <ignition/msgs/light.pb.h>
#include <ignition/msgs/Utility.hh>

#include <ignition/gazebo/components/Light.hh>
#include <ignition/gazebo/components/LightCmd.hh>
#include <ignition/gazebo/components/Name.hh>

#include <sdf/Light.hh>
#include <ignition/gazebo/Conversions.hh>

using namespace ignition;
using namespace ignition::gazebo;

// Find all light entities
void LightControl::FindLightEntities(EntityComponentManager &_ecm)
{
  this->lightEntites.clear();

  // check for such components which has Light, Name components
  _ecm.Each<components::Light, components::Name>(
      [&](const Entity &_entity,
          const components::Light *,
          const components::Name *_name) -> bool
      {
        if (_name->Data() == "led")
        {
          this->lightEntites.push_back(_entity);
          return true;
        }
        return false;
      });
}

// ---------------------------------------------------------------------
void LightControl::PreUpdate(const UpdateInfo &_info,
                             EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  static uint64_t frameCount = 0;
  frameCount++;

  this->FindLightEntities(_ecm);
  if (this->lightEntites.empty())
    return;

  // Animated RGB in [0,1]
  double r = 0.0;
  double g = 0.0;
  const double b = 0.0;
  if ((frameCount / 2000) % 2 == 0){
    r = 1.0;
    g = 0.0;
  }else{
    r = 0.0;
    g = 1.0;
  } 
  ignition::math::Color newColor(r, g, b, 1.0);

  for (const Entity e : this->lightEntites)
  {
    // read data of Light
    auto lightComp = _ecm.Component<components::Light>(e);
    if (!lightComp)
      continue;

    const sdf::Light &sdfLight = lightComp->Data();

    // convert sdf light msg to ignition light msg
    ignition::msgs::Light msg = ignition::gazebo::convert<ignition::msgs::Light>(sdfLight);

    // using Set() to set the fields of light msg
    ignition::msgs::Set(msg.mutable_diffuse(),  newColor);
    ignition::msgs::Set(msg.mutable_specular(), newColor);

    // method2
    _ecm.SetComponentData<components::LightCmd>(e, msg);

    // in case of light we need to trigger update so that rendering system knows it updated
    _ecm.SetChanged(e,
                    components::LightCmd::typeId,
                    ComponentState::PeriodicChange);
  }
}

// Register the plugin
IGNITION_ADD_PLUGIN(
    LightControl,
    ignition::gazebo::System,
    LightControl::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LightControl, "ignition::gazebo::LightControl")