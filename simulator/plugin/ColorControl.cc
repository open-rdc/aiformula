#include "ColorControl.hh"

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

void ColorControl::FindColorEntities(EntityComponentManager &_ecm)
{
  this->ColorEntities.clear();

  _ecm.Each<components::Visual, components::Name>(
      [&](const Entity &_entity,
          const components::Visual *,
          const components::Name *_name) -> bool
      {
        if (_name->Data() == "screen_visual")
        {
          this->ColorEntities.push_back(_entity);
        }
        return true;
      });
  
}

void ColorControl::PreUpdate(const UpdateInfo &_info,
                             EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  std::cout << "[ColorControl] PreUpdate Loop Pythonic Test!" << std::endl;

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

  if (this->ColorEntities.empty())
  {  
    this->FindColorEntities(_ecm);
    if (this->ColorEntities.empty())
      return;
  }

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


  for (const Entity e : this->ColorEntities)
  {
    std::cout << "Set" << color << "!!!!" << std::endl;

    ignition::msgs::Visual visual;
    auto *mat = visual.mutable_material();

    ignition::msgs::Set(mat->mutable_ambient(), newColor);
    ignition::msgs::Set(mat->mutable_diffuse(), newColor);
    ignition::msgs::Set(mat->mutable_specular(), newColor);
    ignition::msgs::Set(mat->mutable_emissive(), newColor);
    auto cmdComp = _ecm.Component<components::VisualCmd>(e);
    if (!cmdComp)
      _ecm.CreateComponent(e, components::VisualCmd(visual));
    else
    {

      cmdComp->Data() = visual;


    }
    auto materialComp = _ecm.Component<components::Material>(e);


    _ecm.SetComponentData<components::VisualCmd>(e, visual);
    
    _ecm.SetChanged(e,
                     components::VisualCmd::typeId,
                     ComponentState::OneTimeChange);
  }
}


IGNITION_ADD_PLUGIN(
    ColorControl,
    ignition::gazebo::System,
    ColorControl::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ColorControl, "ignition::gazebo::ColorControl")