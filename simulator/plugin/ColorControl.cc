#include "ColorControl.hh"

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Material.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/VisualCmd.hh>

// #include <ignition/gazebo/components/Name.hh>
// #include <sdf/Material.hh>
#include <ignition/msgs/visual.pb.h>

using namespace ignition;
using namespace gazebo;

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

  static uint64_t frameCount = 0;
  frameCount++;
  // double simTimeInSeconds = std::chrono::duration_cast<std::chrono::duration<double>>(_info.simTime).count();
  if (this->ColorEntities.empty())
  {  
    this->FindColorEntities(_ecm);
    if (this->ColorEntities.empty())
      return;
  }
  if (frameCount % 1000 == 0)
  {
    ignmsg << "Plugin is looping! Frame Count: " << frameCount 
           << " | Gazebo SimTime: " << _info.simTime.count() << " ns\n";
  }
  double r = 0.0;
  double g = 0.0;
  const double b = 0.0;
  std::string color = "";

  if ((frameCount / 2000) % 2 == 0){
    r = 1.0;
    g = 0.0;
    color = "Red";
  }else{
    r = 0.0;
    g = 1.0;
    color = "Green";
  } 

  ignition::math::Color newColor(r, g, b, 1.0);


  for (const Entity e : this->ColorEntities)
  {
    std::cout << "Set" << color << "!!!!" << std::endl;

    // auto cmd = _ecm.Component<components::VisualCmd>(e);

    // std::cout << "VisualCmd exists = "
    //       << (cmd != nullptr)
    //       << std::endl;

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