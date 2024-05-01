#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <proj.h>
class GNSSPathPublisher : public rclcpp::Node {
public:
    GNSSPathPublisher() : Node("gnss_path_publisher") {
        // パラメータの取得
        this->declare_parameter<std::string>("file_path", "/home/ros2_ws/src/AIFormula_private/gnss_navigation/config/course_data/gazebo_shihou_course.csv");
        auto file_path = this->get_parameter("file_path").as_string();
        // CSVファイルの読み込み
        std::ifstream file(file_path);
        std::string line;
        std::vector<Eigen::Vector2d> coordinates;
        
        int cnt=0;
        double base_x;
        double base_y;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> tokens;
            while (std::getline(ss, cell, ',')) {
                tokens.push_back(cell);
            }
            double lat = std::stod(tokens[0]);
            double lon = std::stod(tokens[1]);
            // GPS座標をUTM座標に変換
            auto [x, y] = convertGPStoUTM(lat, lon);
            if (cnt == 0){
                base_x = x;
                base_y = y;
            }

            coordinates.emplace_back(x - base_x, y - base_y);
            cnt++;
   
        }
        // Pathメッセージの初期化
        
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";
        for (const auto& coord : coordinates) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "map";
            pose.pose.position.x = coord.x();
            pose.pose.position.y = coord.y();
            path_msg.poses.push_back(pose);
        }
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("gnss_path", 10);
        _pub_timer = this->create_wall_timer(
            std::chrono::seconds(1),
            [this] { _publisher_callback(); }
        );
    }
    void _publisher_callback() {
            publisher_->publish(path_msg);
    }
private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr _pub_timer;
    nav_msgs::msg::Path path_msg;
    std::pair<double, double> convertGPStoUTM(double lat, double lon) {
        if (!(-90 <= lat) || !(lat <= 90) || !(-180 <= lon) || !(lon <= 180)) {
        std::cerr << "Error: Latitude or longitude values are out of valid range." << std::endl;
        return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    }
    PJ *P = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:32654", nullptr);
    PJ_COORD p = proj_coord(lat, lon, 0, 0);
    p = proj_trans(P, PJ_FWD, p);
    proj_destroy(P);
    return {p.xy.x, p.xy.y};
    }
};
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GNSSPathPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
