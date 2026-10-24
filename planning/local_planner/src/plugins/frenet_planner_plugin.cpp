#include "local_planner/plugins/frenet_planner_plugin.hpp"

#include <algorithm>

#include <pluginlib/class_list_macros.hpp>

#include "utilities/utils.hpp"

namespace local_planner
{
namespace
{
using object_detection_msgs::msg::ObjectInfo;
}

void FrenetPlannerPlugin::initialize(
    const rclcpp::Logger &,
    const rclcpp::Clock::SharedPtr &,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & params)
{
    params_.horizon_m = params->get_parameter("local_path_horizon_m").as_double();
    params_.resample_interval_m = params->get_parameter("local_path_resample_interval_m").as_double();
    params_.tread_m = params->get_parameter("tread").as_double();
    params_.detection_forward_distance_m = params->get_parameter("avoidance_detection_forward_distance_m").as_double();
    params_.hard_margin_m = params->get_parameter("avoidance_hard_margin_m").as_double();
    params_.soft_margin_m = params->get_parameter("avoidance_soft_margin_m").as_double();
    params_.cluster_gap_m = params->get_parameter("obstacle_cluster_gap_m").as_double();
    params_.max_shift_m = params->get_parameter("max_avoidance_shift_m").as_double();
    params_.lateral_sample_step_m = params->get_parameter("frenet.lateral_sample_step_m").as_double();
    params_.collision_check_margin_m = params->get_parameter("frenet.collision_check_margin_m").as_double();
    params_.avoidance_target_lengths_m = params->get_parameter("frenet.avoidance_target_lengths_m").as_double_array();
    params_.merge_time_s = params->get_parameter("frenet.merge_time_s").as_double();
    params_.merge_length_min_m = params->get_parameter("frenet.merge_length_min_m").as_double();
    params_.merge_length_max_m = params->get_parameter("frenet.merge_length_max_m").as_double();
    params_.merge_lateral_accel_mps2 = params->get_parameter("frenet.merge_lateral_accel_mps2").as_double();
    params_.weight_curvature = params->get_parameter("frenet.weight_curvature").as_double();
    params_.weight_lateral_deviation = params->get_parameter("frenet.weight_lateral_deviation").as_double();
    params_.weight_side_change = params->get_parameter("frenet.weight_side_change").as_double();
    params_.kappa_max = params->get_parameter("frenet.kappa_max").as_double();
    params_.stop_standoff_m = params->get_parameter("stop_standoff_m").as_double();
}

void FrenetPlannerPlugin::setGlobalPath(const nav_msgs::msg::Path & global_path)
{
    global_path_header_ = global_path.header;
    std::vector<frenet::Point2D> points;
    for (const auto & pose : global_path.poses) {
        points.push_back(frenet::Point2D{pose.pose.position.x, pose.pose.position.y});
    }
    reference_.set_points(points);
}

nav_msgs::msg::Path FrenetPlannerPlugin::computeLocalPath(
    const geometry_msgs::msg::Pose & ego_pose,
    const geometry_msgs::msg::TwistWithCovarianceStamped & velocity,
    const object_detection_msgs::msg::ObjectInfoArray & objects)
{
    nav_msgs::msg::Path path;
    if (reference_.size() < 2U) {
        return path;
    }

    // 1. 自車位置を参照線へ射影し、Frenet 座標の初期状態を求める
    const frenet::Pose2D ego{
        ego_pose.position.x, ego_pose.position.y, utils::yaw_from_quaternion(ego_pose.orientation)};
    const frenet::FrenetState initial = frenet::make_initial_state(reference_, ego, params_);
    const double end_s = std::min(initial.s + params_.horizon_m, reference_.length());

    // 2. 信号パネル以外の物体から、経路前方の障害物を抽出する
    std::vector<frenet::Obstacle> obstacle_candidates;
    for (const auto & object : objects.objects) {
        if (object.id != ObjectInfo::ID_PANEL_RED && object.id != ObjectInfo::ID_PANEL_GREEN) {
            obstacle_candidates.push_back(frenet::Obstacle{object.x, object.y, object.width});
        }
    }
    const auto obstacles = frenet::find_obstacles(reference_, initial.s, obstacle_candidates, params_);

    // 3. 合流先 (s, d) の候補を作る
    const auto target_s_list = frenet::make_target_s_list(
        initial, end_s, velocity.twist.twist.linear.x, obstacles, params_);
    const auto target_d_list = frenet::make_target_d_list(obstacles, params_);

    // 4. 候補経路を生成し、制約とコストで最良の経路を選ぶ
    auto points = frenet::select_best_path(
        reference_, initial, end_s, target_s_list, target_d_list, obstacles, previous_points_, params_);

    // 5. 回避できなければ、先頭の障害物の手前で止まる経路にする
    if (points.empty() && !obstacles.empty()) {
        points = frenet::make_stop_path(reference_, initial, obstacles.front(), params_);
    }
    previous_points_ = points;

    // 6. Path メッセージに変換する
    path.header = global_path_header_;
    for (const auto & point : points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.orientation = utils::yaw_to_quaternion(point.yaw);
        path.poses.push_back(pose);
    }
    return path;
}

}

PLUGINLIB_EXPORT_CLASS(local_planner::FrenetPlannerPlugin, local_planner::LocalPlannerPlugin)
