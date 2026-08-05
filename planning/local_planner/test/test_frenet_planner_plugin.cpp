#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "local_planner/plugins/frenet_planner_plugin.hpp"

namespace
{

std::shared_ptr<rclcpp::Node> make_param_node()
{
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides({
        {"local_path_horizon_m", 15.0},
        {"local_path_resample_interval_m", 0.2},
        {"max_centerline_connection_gap_m", 0.5},
        {"vehicle_width_m", 0.6},
        {"avoidance_detection_forward_distance_m", 15.0},
        {"avoidance_hard_margin_m", 0.2},
        {"avoidance_soft_margin_m", 0.3},
        {"envelope_buffer_margin_m", 0.2},
        {"max_avoidance_shift_m", 1.0},
        {"frenet.lateral_sample_step_m", 0.25},
        {"frenet.collision_check_margin_m", 0.2},
        {"frenet.target_lengths_m", std::vector<double>{7.5, 15.0}},
        {"frenet.weight_curvature", 2000.0},
        {"frenet.weight_length", 1.0},
        {"frenet.weight_lateral_deviation", 50.0},
        {"stop_standoff_m", 1.0},
        {"wheelbase", 0.8},
        {"steering_max.pos", 15.0},
    });
    return std::make_shared<rclcpp::Node>("frenet_plugin_test_node", options);
}

nav_msgs::msg::Path make_straight_path()
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (int i = 0; i <= 120; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = 0.5 * i;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }
    return path;
}

geometry_msgs::msg::PoseWithCovarianceStamped make_ego_pose(const double x, const double y)
{
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.orientation.w = 1.0;
    return pose;
}

object_detection_msgs::msg::ObjectInfoArray make_objects(
    const double x, const double y, const double width)
{
    object_detection_msgs::msg::ObjectInfoArray objects;
    objects.header.frame_id = "map";
    object_detection_msgs::msg::ObjectInfo object;
    object.x = static_cast<float>(x);
    object.y = static_cast<float>(y);
    object.width = static_cast<float>(width);
    objects.objects.push_back(object);
    return objects;
}

double y_at_x(const nav_msgs::msg::Path & path, const double x)
{
    double best_y = 0.0;
    double best_distance = std::numeric_limits<double>::max();
    for (const auto & pose : path.poses) {
        const double distance = std::abs(pose.pose.position.x - x);
        if (distance < best_distance) {
            best_distance = distance;
            best_y = pose.pose.position.y;
        }
    }
    return best_y;
}

class FrenetPlannerPluginTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        node_ = make_param_node();
        plugin_.initialize(
            node_->get_logger(), node_->get_clock(), node_->get_node_parameters_interface());
        plugin_.setGlobalPath(make_straight_path());
    }

    std::shared_ptr<rclcpp::Node> node_;
    local_planner::FrenetPlannerPlugin plugin_;
    geometry_msgs::msg::TwistWithCovarianceStamped velocity_;
};

TEST_F(FrenetPlannerPluginTest, ConvergesToCenterlineWithoutObstacle)
{
    const auto path = plugin_.computeLocalPath(make_ego_pose(5.0, 0.4), velocity_, nullptr);
    ASSERT_TRUE(path.has_value());
    ASSERT_GE(path->poses.size(), 2U);
    EXPECT_NEAR(path->poses.front().pose.position.y, 0.4, 0.05);
    EXPECT_NEAR(path->poses.back().pose.position.x, 20.0, 0.3);
    EXPECT_NEAR(path->poses.back().pose.position.y, 0.0, 0.02);
}

TEST_F(FrenetPlannerPluginTest, AvoidsObstacleAwayFromItsSide)
{
    const auto objects = make_objects(12.0, 0.0, 0.0);
    const auto path = plugin_.computeLocalPath(make_ego_pose(5.0, 0.0), velocity_, &objects);
    ASSERT_TRUE(path.has_value());
    EXPECT_NEAR(y_at_x(*path, 12.0), -1.0, 0.05);
    EXPECT_NEAR(path->poses.back().pose.position.y, -1.0, 0.05);
}

TEST_F(FrenetPlannerPluginTest, ReturnsToCenterlineAfterPassingObstacle)
{
    const auto objects = make_objects(12.0, 0.0, 0.0);
    const auto path = plugin_.computeLocalPath(make_ego_pose(14.0, -1.0), velocity_, &objects);
    ASSERT_TRUE(path.has_value());
    EXPECT_NEAR(path->poses.front().pose.position.y, -1.0, 0.05);
    EXPECT_NEAR(path->poses.back().pose.position.y, 0.0, 0.05);
}

TEST_F(FrenetPlannerPluginTest, IgnoresObstacleBeyondDetectionRange)
{
    const auto objects = make_objects(25.0, 0.0, 0.2);
    const auto path = plugin_.computeLocalPath(make_ego_pose(5.0, 0.0), velocity_, &objects);
    ASSERT_TRUE(path.has_value());
    EXPECT_NEAR(y_at_x(*path, 12.0), 0.0, 0.02);
    EXPECT_NEAR(path->poses.back().pose.position.y, 0.0, 0.02);
}

TEST_F(FrenetPlannerPluginTest, StopsShortOfUnavoidableObstacle)
{
    const auto objects = make_objects(12.0, 0.0, 2.0);
    const auto path = plugin_.computeLocalPath(make_ego_pose(5.0, 0.0), velocity_, &objects);
    ASSERT_TRUE(path.has_value());
    ASSERT_GE(path->poses.size(), 2U);
    EXPECT_NEAR(path->poses.back().pose.position.x, 11.0, 0.25);
    for (const auto & pose : path->poses) {
        EXPECT_LE(pose.pose.position.x, 11.05);
    }
}

TEST_F(FrenetPlannerPluginTest, FallsBackToCenterlineWhenCurvatureLimitCannotBeSatisfied)
{
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides({
        {"local_path_horizon_m", 15.0},
        {"local_path_resample_interval_m", 0.2},
        {"max_centerline_connection_gap_m", 0.5},
        {"vehicle_width_m", 0.6},
        {"avoidance_detection_forward_distance_m", 15.0},
        {"avoidance_hard_margin_m", 0.2},
        {"avoidance_soft_margin_m", 0.3},
        {"envelope_buffer_margin_m", 0.2},
        {"max_avoidance_shift_m", 1.0},
        {"frenet.lateral_sample_step_m", 0.25},
        {"frenet.collision_check_margin_m", 0.2},
        {"frenet.target_lengths_m", std::vector<double>{2.0}},
        {"frenet.weight_curvature", 2000.0},
        {"frenet.weight_length", 1.0},
        {"frenet.weight_lateral_deviation", 50.0},
        {"stop_standoff_m", 1.0},
        {"wheelbase", 0.8},
        {"steering_max.pos", 15.0},
    });
    auto short_target_node = std::make_shared<rclcpp::Node>("frenet_plugin_short_target_test_node", options);

    local_planner::FrenetPlannerPlugin plugin;
    plugin.initialize(
        short_target_node->get_logger(), short_target_node->get_clock(),
        short_target_node->get_node_parameters_interface());
    plugin.setGlobalPath(make_straight_path());

    geometry_msgs::msg::TwistWithCovarianceStamped velocity;
    const auto path = plugin.computeLocalPath(make_ego_pose(5.0, 4.0), velocity, nullptr);
    ASSERT_TRUE(path.has_value());
    ASSERT_GE(path->poses.size(), 2U);
    EXPECT_NEAR(path->poses.back().pose.position.y, 0.0, 0.1);
}

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
