#include "LightControl.hh"

#include <ignition/plugin/Register.hh>

#include <ignition/msgs/light.pb.h>
#include <ignition/msgs/Utility.hh>

#include <ignition/gazebo/components/Light.hh>
#include <ignition/gazebo/components/LightCmd.hh>
#include <ignition/gazebo/components/Name.hh>

#include <sdf/Light.hh>
#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Pose.hh>


using namespace ignition;
using namespace ignition::gazebo;

namespace {
  const std::string ROBOT_MODEL_NAME = "ai_car1";
  const ignition::math::Vector3d TARGET_POSITION(80.0, 5.0, 0.0);
  const double DETECTION_RADIUS = 10.0;

  Entity robotEntity = kNullEntity;
  bool target_reached = false;
  std::chrono::steady_clock::duration reach_time;
  bool timer_started = false;
  bool color_changed = false;
}

// Find all light entities
void LightControl::FindLightEntities(EntityComponentManager &_ecm)
{
  this->lightEntites.clear();

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

void LightControl::PreUpdate(const UpdateInfo &_info,
                             EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (robotEntity == kNullEntity)
  {
    _ecm.Each<components::Model, components::Name>(
        [&](const Entity &_entity, const components::Model *, const components::Name *_name) -> bool
        {
          if (_name->Data() == ROBOT_MODEL_NAME)
          {
            robotEntity = _entity;
            return false;
          }
          return true;
        });
  }

  this->FindLightEntities(_ecm);
  if (this->lightEntites.empty())
    return;

  double distance = 20.0;
  if (robotEntity != kNullEntity)
  {
    auto poseComp = _ecm.Component<components::Pose>(robotEntity);
    if (poseComp)
    {
      ignition::math::Vector3d currentPos = poseComp->Data().Pos();

      distance = currentPos.Distance(TARGET_POSITION);

      if (distance <= DETECTION_RADIUS)
      {
        if (!timer_started)
        {
          reach_time = _info.simTime;
          timer_started = true;
          ignmsg << "Robot reached target point! Color change in 6 seconds. SimTime: " 
               << std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count() << "s\n";
        }
      }
    }
  }

  double r = 1.0;
  double g = 0.0;
  const double b = 0.0;
  std::string color = "Red";

  if (timer_started)
  {
    auto elapsed = _info.simTime - reach_time;
    if (elapsed >= std::chrono::seconds(6))
    {
      r = 0.0;
      g = 1.0;
      color = "Green";
      if (!color_changed)
      {
        ignmsg << "6 seconds elapsed since target reach. Changing color to Green! SimTime: " 
               << std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count() << "s\n";
        color_changed = true;
      }
    }
  }
  if (!(distance <= DETECTION_RADIUS))
  {
    r = 1.0;
    g = 0.0;
    color = "Red";
    timer_started = false;
    color_changed = false;
  }

  ignition::math::Color newColor(r, g, b, 1.0);

  for (const Entity e : this->lightEntites)
  {
    auto lightComp = _ecm.Component<components::Light>(e);
    if (!lightComp)
      continue;

    const sdf::Light &sdfLight = lightComp->Data();

    ignition::msgs::Light msg = ignition::gazebo::convert<ignition::msgs::Light>(sdfLight);

    ignition::msgs::Set(msg.mutable_diffuse(),  newColor);
    ignition::msgs::Set(msg.mutable_specular(), newColor);

    _ecm.SetComponentData<components::LightCmd>(e, msg);

    _ecm.SetChanged(e,
                    components::LightCmd::typeId,
                    ComponentState::PeriodicChange);
  }
}

IGNITION_ADD_PLUGIN(
    LightControl,
    ignition::gazebo::System,
    LightControl::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LightControl, "ignition::gazebo::LightControl")