#include "gnssnav/path_publisher_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace gnssnav{

Publisher::Publisher(const rclcpp::NodeOptions& options)
    :Node("path_publisher_node"),
    init_flag_(true)
{
    initCommunication();
    loadCSV();
    path_msg_ = setMsg(xs_, ys_);
    origin_path_msg_ = setMsg(origin_xs_, origin_ys_);

    this->get_parameter("path_publish_freq", freq);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(freq),
        std::bind(&Publisher::loop, this));
}

// pub, param config
void Publisher::initCommunication(void){
    publisher_ = this->create_publisher<nav_msgs::msg::Path>("gnss_path", 10);
    origin_publisher_ = this->create_publisher<nav_msgs::msg::Path>("origin_gnss_path", 10);

    //file_path_ = this->get_parameter("file_path").as_string();
    file_path_ = ament_index_cpp::get_package_share_directory("main_executor")+"/config/"+"course_data/"+"cit-start-BackGate-output.csv";
}

// load CSV file
void Publisher::loadCSV(void){
    std::ifstream file(file_path_);
    printf("loadCSV: file_path_ = %s\n", file_path_.c_str());

    if (!file.is_open()) {
        std::cerr << "loadCSV is failed to open file" << file_path_ << std::endl;
        return;
    }else {
        std::cerr << "loadCSV is sucsess to open file" << std::endl;
    }

    while(std::getline(file, line_)) {
        std::stringstream ss(line_);
        tokens_.clear();
        while(std::getline(ss, cell_, ',')) {
            tokens_.push_back(cell_);
        }
        double lat = std::stod(tokens_[0]);
        double lon = std::stod(tokens_[1]);
        auto [x, y] = convertGPStoUTM(lat, lon);

        if(init_flag_) setInitPose(x, y);

        xs_.push_back(x - base_x_);
        ys_.push_back(y - base_y_);
        origin_xs_.push_back(x);
        origin_ys_.push_back(y);
    }
}

// init pose
void Publisher::setInitPose(double x, double y){
    base_x_ = x;
    base_y_ = y;
    init_flag_ = false;
}

// path create
nav_msgs::msg::Path Publisher::setMsg(const std::vector<double>& xs, const std::vector<double>& ys){
    std::vector<Eigen::Vector2d> spline_points = interpolateSpline(xs, ys, 100);

    nav_msgs::msg::Path path_msg;
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

    return path_msg;
}

// spline
std::vector<Eigen::Vector2d> Publisher::interpolateSpline(const std::vector<double>& xs, const std::vector<double>& ys, int num_points){
    Eigen::Matrix<double, Eigen::Dynamic, 2> points(xs.size(), 2);
    for (size_t i=0; i < xs.size(); ++i){
        points(i, 0) = xs[i];
        points(i, 1) = ys[i];
    }

    auto spline = Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(points.transpose(), 2); //2次のキュービックスプライン

    if(num_points > 1)
        step = 1.0 / (num_points -1);
    for (int i = 0; i < num_points; ++i) {
        double u = i * step;
        Eigen::Vector2d pt = spline(u);
        result_.push_back(pt);
    }
    return result_;
}

// WGS84系からUTM座標系へ変換
std::pair<double, double> Publisher::convertGPStoUTM(double lat, double lon) {
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

void Publisher::loop(void){
    publisher_->publish(path_msg_);
    origin_publisher_->publish(origin_path_msg_);
}

}  // namespace gnssnav
