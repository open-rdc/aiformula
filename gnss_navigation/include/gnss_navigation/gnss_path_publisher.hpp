#ifndef GNSS_PATH_PUBLISHER_HPP
#define GNSS_PATH_PUBLISHER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <proj.h>
#include <unsupported/Eigen/Splines>

namespace gnss_navigation
{

class GNSSPathPublisher : public rclcpp::Node
{
    public:
        GNSSPathPublisher();
        ~GNSSPathPublisher();

        void loop(void);
        int freq_;
    private:
        std::string file_path_;
        std::string line_;
        std::vector<double> xs_;
        std::vector<double> ys_;
        std::vector<double> origin_xs_;
        std::vector<double> origin_ys_;

        std::string cell_;
        std::vector<std::string> tokens_;

        geometry_msgs::msg::PoseStamped pose_;
        geometry_msgs::msg::PoseStamped origin_pose_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr origin_publisher_;
        nav_msgs::msg::Path path_msg_;
        nav_msgs::msg::Path origin_path_msg_;
        rclcpp::TimerBase::SharedPtr _pub_timer;

        std::pair<double, double> convertGPStoUTM(double lat, double lon);
        std::vector<Eigen::Vector2d> interpolateSpline(const std::vector<double>& xs, const std::vector<double>& ys, int num_points);
        std::vector<Eigen::Vector2d> result_;

        void declareParameter(void);
        void initCommunication(void);
        void loadCSV(void);
        void setMsg(void);
        void setOriginMsg(void);
        void _publisher_callback();
        void setInitPose(double x, double y);

        bool init_flag_;
        double base_x_;
        double base_y_;
};

} // namespace gnss_navigation

#endif // GNSS_PATH_PUBLISHER_HPP