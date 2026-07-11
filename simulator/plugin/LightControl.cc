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


// Find all light entities
void LightControl::FindLightEntities(EntityComponentManager &_ecm)
{
  this->LightEntities.clear();
  _ecm.Each<components::Light, components::Name>(
      [&](const Entity &_entity,
          const components::Light *,
          const components::Name *_name) -> bool
      {
        if (_name->Data() == this->LIGHT_ENTITY_NAME)
        {
          this->LightEntities.push_back(_entity);
          return false;
        }
        return true;
      });
}

void LightControl::FindModelEntities(EntityComponentManager &_ecm)
{
  _ecm.Each<components::Model, components::Name>(
      [&](const Entity &_entity, const components::Model *, const components::Name *_name) -> bool
      {
        if (_name->Data() == this->ROBOT_MODEL_NAME)
        {
          this->RobotEntity = _entity;
          return false;
        }
        return true;
      });
}

void LightControl::SetGreen()
{
  this->r = 0.0;
  this->g = 1.0;
  this->color = "Green";
}

void LightControl::SetRed()
{
  this->r = 1.0;
  this->g = 0.0;
  this->color = "Red";
}

void LightControl::TimerReset()
{
  this->timer_started = false;
  this->color_changed = false;
}


void LightControl::PreUpdate(const UpdateInfo &_info,
                             EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (this->RobotEntity == kNullEntity)
  {
    this->FindModelEntities(_ecm);
    if (this->RobotEntity == kNullEntity)
      return;
  }

  this->FindLightEntities(_ecm);
  if (this->LightEntities.empty())
    return;

  if (this->RobotEntity != kNullEntity)
  {
    auto poseComp = _ecm.Component<components::Pose>(this->RobotEntity);
    if (poseComp)
    {
      ignition::math::Vector3d currentPos = poseComp->Data().Pos();
      this->distance = currentPos.Distance(this->TARGET_POSITION);

      if (this->distance <= this->DETECTION_RADIUS)
      {
        if (!this->timer_started)
        {
          this->reach_time = _info.simTime;
          this->timer_started = true;
          ignmsg << "Robot reached target point! Color change in 6 seconds. SimTime: " 
               << std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count() << "s\n";
        }
      }
    }
  }

  if (this->timer_started)
  {
    auto elapsed = _info.simTime - this->reach_time;
    if (elapsed >= std::chrono::seconds(6))
    {
      this->SetGreen();
      if (!this->color_changed)
      {
        ignmsg << "6 seconds elapsed since target reach. Changing color to Green! SimTime: " 
               << std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count() << "s\n";
        this->color_changed = true;
      }
    }
  }
  if (!(this->distance <= this->DETECTION_RADIUS))
  {
    this->SetRed();
    this->TimerReset();
  }

  ignition::math::Color newColor(this->r, this->g, this->b, 1.0);
  
  for (const Entity e : this->LightEntities)
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