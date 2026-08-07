#include "ColorControl.hh"

#include <memory>

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Material.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/VisualCmd.hh>
#include <ignition/msgs/visual.pb.h>

#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Name.hh>

using namespace ignition;
using namespace gazebo;

void ColorControl::Configure(const Entity &_entity,
                              const std::shared_ptr<const sdf::Element> &_sdf,
                              EntityComponentManager &_ecm,
                              EventManager &_eventMgr)
{
  time = (_sdf && _sdf->HasElement("duration_time")) ? _sdf->Get<int>("duration_time") : 10;
  ROBOT_MODEL_NAME = (_sdf && _sdf->HasElement("robot_name")) ? _sdf->Get<std::string>("robot_name") : "model";
  COLOR_ENTITY_NAME = (_sdf && _sdf->HasElement("control_name")) ? _sdf->Get<std::string>("control_name") : "screen_visual";
  TARGET_POSITION = (_sdf && _sdf->HasElement("target_position")) ? _sdf->Get<ignition::math::Vector3d>("target_position") : ignition::math::Vector3d(0.0, 0.0, 0.0);
  DETECTION_RADIUS = (_sdf && _sdf->HasElement("detection_radius")) ? _sdf->Get<double>("detection_radius") : 0.0;
  color_value = (_sdf && _sdf->HasElement("color")) ? _sdf->Get<ignition::math::Color>("color") : ignition::math::Color(0.0, 0.0, 0.0, 1.0);
  color_name = (_sdf && _sdf->HasElement("color_name")) ? _sdf->Get<std::string>("color_name") : "None";
  r = color_value.R();
  g = color_value.G();
  b = color_value.B();
}

void ColorControl::FindColorEntities(EntityComponentManager &_ecm)
{
  ColorEntities.clear();
  _ecm.Each<components::Visual, components::Name>(
      [&](const Entity &_entity,
          const components::Visual *,
          const components::Name *_name) -> bool
      {
        if (_name->Data() == COLOR_ENTITY_NAME)
        {
          ColorEntities.push_back(_entity);
          return false;
        }
        return true;
      });
}

void ColorControl::FindModelEntities(EntityComponentManager &_ecm)
{
  _ecm.Each<components::Model, components::Name>(
      [&](const Entity &_entity,
          const components::Model *,
          const components::Name *_name) -> bool
      {
        if (_name->Data() == ROBOT_MODEL_NAME)
        {
          RobotEntity = _entity;
          return false;
        }
        return true;
      });
}

void ColorControl::SetGreen()
{
  r = 0.0;
  g = 1.0;
  b = 0.0;
  color_name = "Green";
}

void ColorControl::SetRed()
{
  r = 1.0;
  g = 0.0;
  b = 0.0;
  color_name = "Red";
}

void ColorControl::TimerReset()
{
  timer_started = false;
  color_changed = false;
}


void ColorControl::PreUpdate(const UpdateInfo &_info,
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

  if (ColorEntities.empty())
  {  
    FindColorEntities(_ecm);
    if (ColorEntities.empty())
      return;
  }

  if (RobotEntity != kNullEntity)
  {
    auto poseComp = _ecm.Component<components::Pose>(RobotEntity);
    if (poseComp)
    {
      ignition::math::Vector3d currentPos = poseComp->Data().Pos();
      
      x = currentPos.X();
      y = currentPos.Y();

      if (x < 85.0 && x > 77.0 && y < 5.0 && y > 0.0)
      {
        if (!timer_started)
        {
          reach_time = _info.simTime;
          timer_started = true;
          std::cout << "Robot reached target point! Color change in " << time << " seconds. SimTime: " 
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
        std::cout << time << " seconds elapsed since target reach. Changing color to Green! SimTime: " 
               << std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count() << "s\n";
        color_changed = true;
      }
    }
  }
  if (!(x < 85.0 && x > 77.0 && y < 5.0 && y > 0.0))
  {
    SetRed();
    TimerReset();
  }

  ignition::math::Color newColor(r, g, b, 1.0);

  for (const Entity e : ColorEntities)
  {
    ignition::msgs::Visual visual;
    auto *mat = visual.mutable_material();

    ignition::msgs::Set(mat->mutable_ambient(), newColor);
    ignition::msgs::Set(mat->mutable_diffuse(), newColor);
    ignition::msgs::Set(mat->mutable_specular(), newColor);
    ignition::msgs::Set(mat->mutable_emissive(), newColor);

    _ecm.SetComponentData<components::VisualCmd>(e, visual);
    
    _ecm.SetChanged(e,
                     components::VisualCmd::typeId,
                     ComponentState::OneTimeChange);
  }
}

IGNITION_ADD_PLUGIN(
    ColorControl,
    ignition::gazebo::System,
    ColorControl::ISystemConfigure,
    ColorControl::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ColorControl, "ignition::gazebo::ColorControl")