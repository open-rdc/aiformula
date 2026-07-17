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


void LightControl::Configure(const Entity &_entity,
                              const std::shared_ptr<const sdf::Element> &_sdf,
                              EntityComponentManager &_ecm,
                              EventManager &_eventMgr)
{
  time = (_sdf && _sdf->HasElement("duration")) ? _sdf->Get<int>("duration") : 10;
  ROBOT_MODEL_NAME = (_sdf && _sdf->HasElement("robot_name")) ? _sdf->Get<std::string>("robot_name") : "model";
  LIGHT_ENTITY_NAME = (_sdf && _sdf->HasElement("control_name")) ? _sdf->Get<std::string>("control_name") : "led";
  TARGET_POSITION = (_sdf && _sdf->HasElement("target_position")) ? _sdf->Get<ignition::math::Vector3d>("target_position") : ignition::math::Vector3d(0.0, 0.0, 0.0);
  DETECTION_RADIUS = (_sdf && _sdf->HasElement("detection_radius")) ? _sdf->Get<double>("detection_radius") : 0.0;
  color_value = (_sdf && _sdf->HasElement("color")) ? _sdf->Get<ignition::math::Color>("color") : ignition::math::Color(0.0, 0.0, 0.0, 1.0);
  color_name = (_sdf && _sdf->HasElement("color_name")) ? _sdf->Get<std::string>("color_name") : "None";
  r = color_value.R();
  g = color_value.G();
  b = color_value.B();
}

void LightControl::FindLightEntities(EntityComponentManager &_ecm)
{
  LightEntities.clear();
  _ecm.Each<components::Light, components::Name>(
      [&](const Entity &_entity,
          const components::Light *,
          const components::Name *_name) -> bool
      {
        if (_name->Data() == LIGHT_ENTITY_NAME)
        {
          LightEntities.push_back(_entity);
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
        if (_name->Data() == ROBOT_MODEL_NAME)
        {
          RobotEntity = _entity;
          return false;
        }
        return true;
      });
}

void LightControl::SetGreen()
{
  r = 0.0;
  g = 1.0;
  color_name = "Green";
}

void LightControl::SetRed()
{
  r = 1.0;
  g = 0.0;
  color_name = "Red";
}

void LightControl::TimerReset()
{
  timer_started = false;
  color_changed = false;
}


void LightControl::PreUpdate(const UpdateInfo &_info,
                             EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (RobotEntity == kNullEntity)
  {
    FindModelEntities(_ecm);
    if (RobotEntity == kNullEntity)
      return;
  }

  FindLightEntities(_ecm);
  if (LightEntities.empty())
    return;

  if (RobotEntity != kNullEntity)
  {
    auto poseComp = _ecm.Component<components::Pose>(RobotEntity);
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
          ignmsg << "Robot reached target point! Color change in " << time << " seconds. SimTime: " 
               << std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count() << "s\n";
        }
      }
    }
  }

  if (timer_started)
  {
    auto elapsed = _info.simTime - reach_time;
    if (elapsed >= std::chrono::seconds(time))
    {
      SetGreen();
      if (!color_changed)
      {
        ignmsg << time << " seconds elapsed since target reach. Changing color to Green! SimTime: " 
               << std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count() << "s\n";
        color_changed = true;
      }
    }
  }
  if (!(distance <= DETECTION_RADIUS))
  {
    SetRed();
    TimerReset();
  }

  ignition::math::Color newColor(r, g, b, 1.0);
  
  for (const Entity e : LightEntities)
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
    LightControl::ISystemConfigure,
    LightControl::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LightControl, "ignition::gazebo::LightControl")