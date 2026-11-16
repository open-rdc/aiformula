#include "controller/controller_node.hpp"
#include "ekf_localizer/ekf_localizer_node.hpp"
#include "ekf_localizer/map_odom_tf_node.hpp"
#include "ekf_localizer/odom_tf_node.hpp"
#include "lane_line_publisher/lane_line_publisher_node.hpp"
#include "lane_line_publisher/vectormap_visualizer_node.hpp"
#include "local_planner/local_planner_server.hpp"
#include "mission_planner/mission_planner_node.hpp"
#include "object_detection/object_detector_node.hpp"
#include "path_smoother/path_smoother_node.hpp"
#include "pose_estimater/pose_estimater_node.hpp"
#include "road_detector/road_detector_node.hpp"
#include "speed_path_planner/speed_path_planner_node.hpp"
#include "topomap_localizer/topomap_localizer_node.hpp"
#include "topomap_server/topomap_server_node.hpp"
#include "traffic_signal_stop/traffic_signal_stop_node.hpp"
#include "trajectory_follower/controller_server.hpp"
#include "vector_e2e_planner/vector_e2e_planner_node.hpp"
#include "vectormap_server/vectormap_server_node.hpp"
#include "vision_lane_planner/vision_lane_planner_node.hpp"
#include "zed_wrapper/zed_wrapper_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;

    rclcpp::NodeOptions nodes_option;
    nodes_option.allow_undeclared_parameters(true);
    nodes_option.automatically_declare_parameters_from_overrides(true);
    nodes_option.use_intra_process_comms(true);
    bool        use_sim = false;
    bool        mapless = false;
    std::string course;
    std::string planning;
    {
        auto launch = rclcpp::Node("launch", nodes_option);
        use_sim  = launch.get_parameter("sim").as_bool();
        mapless  = launch.get_parameter("mapless.use").as_bool();
        course   = launch.get_parameter("course").as_string();
        planning = launch.get_parameter("mapless.planning").as_string();
    }
    const bool blue = mapless && course == "blue";
    if (planning != "vision_planner" && planning != "vector_planner") {
        RCLCPP_FATAL(rclcpp::get_logger("main_executor"), "mapless.planning は \"vision_planner\" か \"vector_planner\" を指定してください: \"%s\"", planning.c_str());
        rclcpp::shutdown();
        return 1;
    }
    const bool use_vector_planner = mapless && planning == "vector_planner";
#ifndef ENABLE_TENSORRT
    if (use_vector_planner) {
        RCLCPP_FATAL(rclcpp::get_logger("main_executor"), "mapless.planning=vector_planner には TensorRT が必要です");
        rclcpp::shutdown();
        return 1;
    }
#endif

    if (use_sim) {
        nodes_option.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    }


    auto lane_line_publisher_node  = std::make_shared<lane_line_publisher::LaneLinePublisherNode>(nodes_option);
    auto vectormap_visualizer_node = std::make_shared<lane_line_publisher::VectormapVisualizerNode>(nodes_option);
    auto vectormap_server_node     = mapless ? nullptr : std::make_shared<vectormap_server::VectormapServerNode>(nodes_option);
    auto pose_estimater_node       = mapless ? nullptr : std::make_shared<pose_estimater::PoseEstimaterNode>(nodes_option);
    auto ekf_localizer_node        = mapless ? nullptr : std::make_shared<ekf_localizer::EkfLocalizerNode>(nodes_option);
    auto map_odom_tf_node          = mapless ? nullptr : std::make_shared<ekf_localizer::MapOdomTfNode>(nodes_option);
    auto topomap_server_node       = (blue && !use_vector_planner) ? std::make_shared<topomap_server::TopomapServerNode>(nodes_option) : nullptr;
    auto topomap_localizer_node    = (blue && !use_vector_planner) ? std::make_shared<topomap_localizer::TopomapLocalizerNode>(nodes_option) : nullptr;
    auto odom_tf_node              = std::make_shared<ekf_localizer::OdomTfNode>(nodes_option);
    auto mission_planner_node      = mapless ? nullptr : std::make_shared<mission_planner::MissionPlannerNode>(nodes_option);
    auto local_planner_server_node = use_vector_planner ? nullptr : std::make_shared<local_planner::LocalPlannerServer>(nodes_option);
    auto speed_path_planner_node   = std::make_shared<speed_path_planner::SpeedPathPlannerNode>(nodes_option);
    auto controller_server_node    = std::make_shared<trajectory_follower::ControllerServer>(nodes_option);
    auto controller_node           = std::make_shared<controller::Controller>(nodes_option);

    exec.add_node(lane_line_publisher_node);
    if (!mapless) {
        exec.add_node(vectormap_server_node);
        exec.add_node(vectormap_visualizer_node);
        exec.add_node(pose_estimater_node);
        exec.add_node(ekf_localizer_node);
        exec.add_node(map_odom_tf_node);
        exec.add_node(mission_planner_node);

    }
    if (blue && !use_vector_planner) {
        exec.add_node(topomap_server_node);
        exec.add_node(topomap_localizer_node);
    }
    exec.add_node(odom_tf_node);
    if (!use_vector_planner) {
        exec.add_node(local_planner_server_node);
    }
    exec.add_node(speed_path_planner_node);
    exec.add_node(controller_server_node);
    // exec.add_node(controller_node);

#ifdef ENABLE_ZED
    std::shared_ptr<zed_wrapper::ZedWrapperNode> zed_wrapper_node;
    if (!use_sim) {
        zed_wrapper_node = std::make_shared<zed_wrapper::ZedWrapperNode>(nodes_option);
        exec.add_node(zed_wrapper_node);
    }
#endif

#ifdef ENABLE_TENSORRT
    auto object_detector_node      = use_vector_planner ? nullptr : std::make_shared<object_detection::ObjectDetectorNode>(nodes_option);
    auto traffic_signal_stop_node = (blue || use_vector_planner) ? nullptr : std::make_shared<traffic_signal_stop::TrafficSignalStopNode>(nodes_option);
    auto road_detector_node       = mapless ? nullptr : std::make_shared<road_detector::RoadDetectorNode>(nodes_option);
    auto vision_lane_planner_node = (mapless && !use_vector_planner) ? std::make_shared<vision_lane_planner::VisionLanePlannerNode>(nodes_option) : nullptr;
    auto path_smoother_node       = (mapless && !use_vector_planner) ? std::make_shared<path_smoother::PathSmootherNode>(nodes_option) : nullptr;
    auto vector_e2e_planner_node  = use_vector_planner ? std::make_shared<vector_e2e_planner::VectorE2ePlannerNode>(nodes_option) : nullptr;

    if (!use_vector_planner) {
        exec.add_node(object_detector_node);
    }
    if (!blue && !use_vector_planner) {
        exec.add_node(traffic_signal_stop_node);
    }
    if (use_vector_planner) {
        exec.add_node(vector_e2e_planner_node);
    } else if (mapless) {
        exec.add_node(vision_lane_planner_node);
        exec.add_node(path_smoother_node);
    } else {
        exec.add_node(road_detector_node);
    }
#endif

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
