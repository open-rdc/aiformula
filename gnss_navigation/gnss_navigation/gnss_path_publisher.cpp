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
        
        // ヘッダー行をスキップする場合は次の行を追加
        // std::getline(file, line);

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> tokens;

            while (std::getline(ss, cell, ',')) {
                tokens.push_back(cell);
            }

            double lon = std::stod(tokens[0]);
            double lat = std::stod(tokens[1]);

            // GPS座標をUTM座標に変換
            auto [x, y] = convertGPStoUTM(lon, lat);
            coordinates.emplace_back(x, y);

            std::cout << "Converted UTM coordinates: X=" << x << ", Y=" << y << std::endl;
        }

        // Pathメッセージの初期化
        nav_msgs::msg::Path path_msg;
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
        publisher_->publish(path_msg);
    }


private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;

    std::pair<double, double> convertGPStoUTM(double lat, double lon) {
        if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
        std::cerr << "Error: Latitude or longitude values are out of valid range." << std::endl;
        return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    }
    PJ *P = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:32654", nullptr);
    if (!P) {
        std::cerr << "Projection creation failed: " << proj_errno_string(proj_errno(PJ_DEFAULT_CTX)) << std::endl;
        return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    }

    PJ_COORD p = proj_coord(lon, lat, 0, 0);
    p = proj_trans(P, PJ_FWD, p);
    if (p.xy.x == HUGE_VAL || p.xy.y == HUGE_VAL) {
        std::cerr << "Invalid conversion for lat=" << lat << ", lon=" << lon
                  << ", error: " << proj_errno_string(proj_errno(P)) << std::endl;
        proj_destroy(P);
        return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    }

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

