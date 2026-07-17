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

ColorControl::ColorControl()
    : time(10),
      ROBOT_MODEL_NAME("model"),
      COLOR_ENTITY_NAME("screen_visual"),
      TARGET_POSITION(0.0, 0.0, 0.0),
      DETECTION_RADIUS(0.0),
      color_value(0.0, 0.0, 0.0, 1.0),
      color_name("None"),
      r(color_value.R()),
      g(color_value.G()),
      b(color_value.B())
{
}

ColorControl::ColorControl(const std::shared_ptr<const sdf::Element> &_sdf)
    : time(_sdf && _sdf->HasElement("duration_time") ? _sdf->Get<int>("duration_time") : 10),
      ROBOT_MODEL_NAME(_sdf && _sdf->HasElement("robot_name") ? _sdf->Get<std::string>("robot_name") : "model"),
      COLOR_ENTITY_NAME(_sdf && _sdf->HasElement("control_name") ? _sdf->Get<std::string>("control_name") : "screen_visual"),
      TARGET_POSITION(_sdf && _sdf->HasElement("target_position") ? _sdf->Get<ignition::math::Vector3d>("target_position") : ignition::math::Vector3d(0.0, 0.0, 0.0)),
      DETECTION_RADIUS(_sdf && _sdf->HasElement("detection_radius") ? _sdf->Get<double>("detection_radius") : 0.0),
      color_value(_sdf && _sdf->HasElement("color") ? _sdf->Get<ignition::math::Color>("color") : ignition::math::Color(0.0, 0.0, 0.0, 1.0)),
      color_name(_sdf && _sdf->HasElement("color_name") ? _sdf->Get<std::string>("color_name") : "None"),
      r(color_value.R()),
      g(color_value.G()),
      b(color_value.B())
{
}

void ColorControl::FindColorEntities(EntityComponentManager &_ecm)
{
  this->ColorEntities.clear();
  _ecm.Each<components::Visual, components::Name>(
      [&](const Entity &_entity,
          const components::Visual *,
          const components::Name *_name) -> bool
      {
        if (_name->Data() == COLOR_ENTITY_NAME)
        {
          // this->ColorEntities.push_back(_entity);
          this->ColorEntities.push_back(_entity);
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
          this->RobotEntity = _entity;
          return false;
        }
        return true;
      });
}

void ColorControl::SetGreen()
{
  this->r = 0.0;
  this->g = 1.0;
  this->b = 0.0;
  this->color_name = "Green"; // Set the color name to "Green"
}

void ColorControl::SetRed()
{
  this->r = 1.0;
  this->g = 0.0;
  this->b = 0.0;
  this->color_name = "Red";
}

void ColorControl::TimerReset()
{
  this->timer_started = false;
  this->color_changed = false;
}


void ColorControl::PreUpdate(const UpdateInfo &_info,
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

  if (this->ColorEntities.empty())
  {  
    this->FindColorEntities(_ecm);
    if (this->ColorEntities.empty())
      return;
  }

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
          std::cout << "Robot reached target point! Color change in 6 seconds. SimTime: " 
                    << std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count() << "s\n";
        }
      }
    }
  }

  if (this->timer_started)
  {
    auto elapsed = _info.simTime - this->reach_time;
    if (elapsed >= std::chrono::seconds(time))
    {
      this->SetGreen();
      if (!this->color_changed)
      {
        std::cout << "6 seconds elapsed since target reach. Changing color to Green! SimTime: " 
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

  for (const Entity e : this->ColorEntities)
  {
    //std::cout << "Set" << this->color_name << "!!!!" << std::endl;

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
    ColorControl::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ColorControl, "ignition::gazebo::ColorControl")