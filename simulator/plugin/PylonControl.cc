#include "PylonControl.hh"

#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/entity.pb.h>
#include <ignition/msgs/entity_factory.pb.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include <proj.h>

using namespace ignition;
using namespace gazebo;

void PylonControl::Configure(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager &)
{
  PATH_FILE_NAME = (_sdf && _sdf->HasElement("path_file_name")) ? _sdf->Get<std::string>("path_file_name") : "dynamic_obstacle_left1";
  PANEL_MODEL_NAME = (_sdf && _sdf->HasElement("panel_model_name")) ? _sdf->Get<std::string>("panel_model_name") : "panel";
  ROBOT_MODEL_NAME = (_sdf && _sdf->HasElement("robot_model_name")) ? _sdf->Get<std::string>("robot_model_name") : "ai_car1";
  PYLON_MODEL_NAME = (_sdf && _sdf->HasElement("pylon_model_name")) ? _sdf->Get<std::string>("pylon_model_name") : "pylon";
  SETTING_SERVICE_NAME = (_sdf && _sdf->HasElement("setting_service_name")) ? _sdf->Get<std::string>("setting_service_name") : "/pylon/setting";
  CLEAR_SERVICE_NAME = (_sdf && _sdf->HasElement("clear_service_name")) ? _sdf->Get<std::string>("clear_service_name") : "/pylon/clear";

  if (_sdf && _sdf->HasElement("clearance")) clearance_ = _sdf->Get<double>("clearance");
  if (_sdf && _sdf->HasElement("group_interval")) group_interval_ = _sdf->Get<double>("group_interval");
  if (_sdf && _sdf->HasElement("lateral_range")) lateral_range_ = _sdf->Get<double>("lateral_range");
  if (_sdf && _sdf->HasElement("pylon_interval")) pylon_interval_ = _sdf->Get<double>("pylon_interval");
  if (_sdf && _sdf->HasElement("group_num")) group_num_ = _sdf->Get<int>("group_num");
  if (_sdf && _sdf->HasElement("min_pylon_num")) min_pylon_num_ = _sdf->Get<int>("min_pylon_num");
  if (_sdf && _sdf->HasElement("max_pylon_num")) max_pylon_num_ = _sdf->Get<int>("max_pylon_num");

  model_file_path_ = ament_index_cpp::get_package_share_directory("simulator") + "/models/" + PYLON_MODEL_NAME + "/model.sdf";

  const auto *world_name = _ecm.Component<components::Name>(_entity);
  if (!world_name)
  {
    ignerr << "PylonControl must be attached to a world" << std::endl;
    return;
  }
  create_service_ = "/world/" + world_name->Data() + "/create";
  remove_service_ = "/world/" + world_name->Data() + "/remove";

  LoadPath();

  context_ = std::make_shared<rclcpp::Context>();
  context_->init(0, nullptr);

  rclcpp::NodeOptions node_options;
  node_options.context(context_);
  ros_node_ = std::make_shared<rclcpp::Node>("pylon_control", node_options);

  setting_service_ = ros_node_->create_service<std_srvs::srv::Empty>(
      SETTING_SERVICE_NAME,
      std::bind(&PylonControl::SettingPylon, this, std::placeholders::_1, std::placeholders::_2));
  clear_service_ = ros_node_->create_service<std_srvs::srv::Empty>(
      CLEAR_SERVICE_NAME,
      std::bind(&PylonControl::ClearPylon, this, std::placeholders::_1, std::placeholders::_2));

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context_;
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);
  executor_->add_node(ros_node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });

  ros_ready_ = true;
}

PylonControl::~PylonControl()
{
  ShutdownRos();
}

void PylonControl::ShutdownRos()
{
  ros_ready_ = false;

  if (context_) context_->shutdown("PylonControl plugin shutdown");
  if (spin_thread_.joinable()) spin_thread_.join();

  executor_.reset();
  setting_service_.reset();
  clear_service_.reset();
  ros_node_.reset();
  context_.reset();
}

void PylonControl::LoadPath()
{
  const std::string file_path = ament_index_cpp::get_package_share_directory("simulator") + "/plugin/config/" + PATH_FILE_NAME + ".csv";
  std::ifstream file(file_path);
  if (!file.is_open())
  {
    ignerr << "Failed to open csv file " << file_path << std::endl;
    return;
  }

  PJ *transform = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:32654", nullptr);
  if (!transform)
  {
    ignerr << "Failed to create coordinate transformation" << std::endl;
    return;
  }

  double base_x = 0.0;
  double base_y = 0.0;
  bool init_flag = true;
  std::string line;
  while (std::getline(file, line))
  {
    std::stringstream ss(line);
    std::string cell;
    std::vector<double> values;
    while (std::getline(ss, cell, ',')) values.push_back(std::stod(cell));
    if (values.size() < 2) continue;

    const PJ_COORD coord = proj_trans(transform, PJ_FWD, proj_coord(values[0], values[1], 0, 0));
    if (init_flag)
    {
      base_x = coord.xy.x;
      base_y = coord.xy.y;
      init_flag = false;
    }
    path_.emplace_back(coord.xy.x - base_x, coord.xy.y - base_y);
  }
  proj_destroy(transform);
}

void PylonControl::FindClearancePositions(EntityComponentManager &_ecm)
{
  for (const std::string &name : {PANEL_MODEL_NAME, ROBOT_MODEL_NAME})
  {
    const Entity entity = _ecm.EntityByComponents(components::Model(), components::Name(name));
    const auto *pose = _ecm.Component<components::Pose>(entity);
    if (!pose)
    {
      clearance_positions_.clear();
      return;
    }
    clearance_positions_.emplace_back(pose->Data().Pos().X(), pose->Data().Pos().Y());
  }
}

std::vector<size_t> PylonControl::SelectPathIndices()
{
  const double clearance = clearance_ + lateral_range_ + (max_pylon_num_ - 1) * 0.5 * pylon_interval_;

  std::vector<size_t> candidates;
  for (size_t i = 1; i + 1 < path_.size(); ++i)
  {
    const bool cleared = std::all_of(clearance_positions_.begin(), clearance_positions_.end(),
        [&](const math::Vector2d &_position) { return path_[i].Distance(_position) > clearance; });
    if (cleared) candidates.push_back(i);
  }
  std::shuffle(candidates.begin(), candidates.end(), random_engine_);

  std::vector<size_t> selected;
  for (const size_t index : candidates)
  {
    if (static_cast<int>(selected.size()) >= group_num_) break;
    const bool separated = std::all_of(selected.begin(), selected.end(),
        [&](const size_t _selected) { return path_[_selected].Distance(path_[index]) > group_interval_; });
    if (separated) selected.push_back(index);
  }
  return selected;
}

void PylonControl::SpawnPylons()
{
  std::uniform_real_distribution<double> offset_dist(-lateral_range_, lateral_range_);
  std::uniform_int_distribution<int> num_dist(min_pylon_num_, max_pylon_num_);

  for (const size_t index : SelectPathIndices())
  {
    const math::Vector2d direction = (path_[index + 1] - path_[index - 1]).Normalized();
    const math::Vector2d normal(-direction.Y(), direction.X());
    const math::Vector2d center = path_[index] + normal * offset_dist(random_engine_);
    const double yaw = std::atan2(direction.Y(), direction.X());

    const int pylon_num = num_dist(random_engine_);
    for (int i = 0; i < pylon_num; ++i)
    {
      const math::Vector2d position = center + normal * ((i - (pylon_num - 1) * 0.5) * pylon_interval_);
      CreatePylon(PYLON_MODEL_NAME + "_" + std::to_string(pylon_count_++), position, yaw);
    }
  }
}

void PylonControl::CreatePylon(const std::string &_name, const math::Vector2d &_position, double _yaw)
{
  msgs::EntityFactory request;
  request.set_sdf_filename(model_file_path_);
  request.set_name(_name);
  request.set_allow_renaming(false);
  msgs::Set(request.mutable_pose(), math::Pose3d(_position.X(), _position.Y(), 0.0, 0.0, 0.0, _yaw));

  msgs::Boolean response;
  bool result = false;
  if (node_.Request(create_service_, request, 1000, response, result) && result && response.data())
    pylon_names_.push_back(_name);
  else
    ignerr << "Failed to create pylon " << _name << std::endl;
}

void PylonControl::RemovePylons()
{
  for (const std::string &name : pylon_names_)
  {
    msgs::Entity request;
    request.set_name(name);
    request.set_type(msgs::Entity::MODEL);

    msgs::Boolean response;
    bool result = false;
    node_.Request(remove_service_, request, 1000, response, result);
  }
  pylon_names_.clear();
}

void PylonControl::SettingPylon(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  setting_requested_ = true;
}

void PylonControl::ClearPylon(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                              std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  clear_requested_ = true;
}

void PylonControl::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  if (_info.paused || !ros_ready_)
    return;

  if (clearance_positions_.empty())
    FindClearancePositions(_ecm);

  if (clear_requested_.exchange(false))
    RemovePylons();

  if (setting_requested_.exchange(false))
  {
    if (path_.empty() || clearance_positions_.empty())
    {
      ignerr << "Failed to setting pylons" << std::endl;
      return;
    }
    RemovePylons();
    SpawnPylons();
  }
}

IGNITION_ADD_PLUGIN(
    PylonControl,
    ignition::gazebo::System,
    PylonControl::ISystemConfigure,
    PylonControl::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(PylonControl, "ignition::gazebo::PylonControl")
