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
        std::vector<Eigen::Vector2d> origin_coordinates;
        std::vector<double> xs;
        std::vector<double> ys;
        std::vector<double> origin_xs;
        std::vector<double> origin_ys;
        
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

            origin_coordinates.emplace_back(x, y);
            origin_xs.push_back(x);
            origin_ys.push_back(y);

            coordinates.emplace_back(x - base_x, y - base_y);
            xs.push_back(x - base_x);
            ys.push_back(y - base_y);
            cnt++;
   
        }
        

        std::vector<Eigen::Vector2d> spline_points = interpolateSpline(xs, ys, 100);  // 100は補間点の数
        std::vector<Eigen::Vector2d> origin_spline_points = interpolateSpline(origin_xs, origin_ys, 100);  // 100は補間点の数

        // Pathメッセージの初期化
        
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";
        for (const auto& coord : spline_points) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "map";
            pose.pose.position.x = coord.x();
            pose.pose.position.y = coord.y();
            path_msg.poses.push_back(pose);
        }
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("gnss_path", 10);

        origin_path_msg.header.stamp = this->now();
        origin_path_msg.header.frame_id = "map";
        for (const auto& origin_coord : origin_spline_points) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "map";
            pose.pose.position.x = origin_coord.x();
            pose.pose.position.y = origin_coord.y();
            origin_path_msg.poses.push_back(pose);
        }
        origin_publisher_ = this->create_publisher<nav_msgs::msg::Path>("origin_gnss_path", 10);

        _pub_timer = this->create_wall_timer(
            std::chrono::seconds(1),
            [this] { _publisher_callback(); }
        );
    }
    void _publisher_callback() {
            publisher_->publish(path_msg);
            origin_publisher_->publish(origin_path_msg);
    }
private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr origin_publisher_;
    rclcpp::TimerBase::SharedPtr _pub_timer;
    nav_msgs::msg::Path path_msg;
    nav_msgs::msg::Path origin_path_msg;
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

    std::vector<Eigen::Vector2d> interpolateSpline(const std::vector<double>& xs, const std::vector<double>& ys, int num_points) {
    Eigen::Matrix<double, Eigen::Dynamic, 2> points(xs.size(), 2);
    for (size_t i = 0; i < xs.size(); ++i) {
        points(i, 0) = xs[i];
        points(i, 1) = ys[i];
    }

    auto spline = Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(points.transpose(), 2); // 2次のキュービックスプライン

    std::vector<Eigen::Vector2d> result;
    double step = 1.0 / (num_points - 1);
    for (int i = 0; i < num_points; ++i) {
        double u = i * step;
        Eigen::Vector2d pt = spline(u);
        result.push_back(pt);
    }
    return result;
    }

};
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GNSSPathPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
