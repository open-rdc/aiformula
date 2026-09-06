#include "ColorControl.hh"

#include <memory>

#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Material.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/VisualCmd.hh>
#include <ignition/msgs/visual.pb.h>

#include <ignition/gazebo/components/Name.hh>

using namespace ignition;
using namespace gazebo;

void ColorControl::Configure(const Entity &_entity,
                              const std::shared_ptr<const sdf::Element> &_sdf,
                              EntityComponentManager &_ecm,
                              EventManager &_eventMgr)
{
  COLOR_ENTITY_NAME = (_sdf && _sdf->HasElement("control_name")) ? _sdf->Get<std::string>("control_name") : "screen_visual";
  SERVICE_NAME = (_sdf && _sdf->HasElement("service_name")) ? _sdf->Get<std::string>("service_name") : "/panel/toggle_signal";

  context_ = std::make_shared<rclcpp::Context>();
  context_->init(0, nullptr);

  rclcpp::NodeOptions node_options;
  node_options.context(context_);
  ros_node_ = std::make_shared<rclcpp::Node>("color_control", node_options);

  service_ = ros_node_->create_service<std_srvs::srv::Empty>(
      SERVICE_NAME,
      std::bind(&ColorControl::ToggleSignal, this, std::placeholders::_1, std::placeholders::_2));

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context_;
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);
  executor_->add_node(ros_node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });

  ros_ready_ = true;
}

ColorControl::~ColorControl()
{
  ShutdownRos();
}

void ColorControl::ShutdownRos()
{
  ros_ready_ = false;

  if (context_) context_->shutdown("ColorControl plugin shutdown");
  if (spin_thread_.joinable()) spin_thread_.join();

  executor_.reset();
  service_.reset();
  ros_node_.reset();
  context_.reset();
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

void ColorControl::ToggleSignal(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  is_green_ = !is_green_.load();
}

void ColorControl::PreUpdate(const UpdateInfo &_info,
                             EntityComponentManager &_ecm)
{
  if (_info.paused || !ros_ready_)
    return;

  if (ColorEntities.empty())
  {
    FindColorEntities(_ecm);
    if (ColorEntities.empty())
      return;
  }

  const ignition::math::Color newColor = is_green_ ? ignition::math::Color(0.0, 1.0, 0.0, 1.0) : ignition::math::Color(1.0, 0.0, 0.0, 1.0);

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
