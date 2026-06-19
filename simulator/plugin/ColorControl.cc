#include "ColorControl.hh"

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Material.hh>
#include <ignition/gazebo/components/Name.hh>
#include <sdf/Material.hh>

using namespace ignition;
using namespace gazebo;

// すべてのマテリアルエンティティを検索
void ColorControl::FindColorEntities(EntityComponentManager &_ecm)
{
  this->ColorEntities.clear();

  _ecm.Each<components::Material, components::Name>(
      [&](const Entity &_entity,
          const components::Material *,
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

double simTimeInSeconds = std::chrono::duration_cast<std::chrono::duration<double>>(_info.simTime).count();
  this->FindColorEntities(_ecm);
  if (this->ColorEntities.empty())
    return;

  double r = 0.0;
  double g = 0.0;
  const double b = 0.0;

  if (static_cast<int>(simTimeInSeconds) % 2 == 0){
    r = 1.0;
    g = 0.0;
  }else{
    r = 0.0;
    g = 1.0;
  } 

  ignition::math::Color newColor(r, g, b, 1.0);

  for (const Entity e : this->ColorEntities)
  {
    auto ColorComp = _ecm.Component<components::Material>(e);
    if (!ColorComp)
      continue;

    // ignition環境でも、この「Data()を直接書き換える方法」が一番きれいに動きます
    sdf::Material &sdfColor = ColorComp->Data();

    sdfColor.SetAmbient(newColor);
    sdfColor.SetDiffuse(newColor);
    sdfColor.SetEmissive(newColor);

    // データの変更を通知
    _ecm.SetChanged(e,
                    components::Material::typeId,
                    ComponentState::PeriodicChange);
  }
}

// Ignition規格の登録マクロ
IGNITION_ADD_PLUGIN(
    ColorControl,
    ignition::gazebo::System,
    ColorControl::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ColorControl, "ignition::gazebo::ColorControl")