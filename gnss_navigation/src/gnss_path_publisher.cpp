#include "gnss_navigation/gnss_path_publisher.hpp"

namespace gnss_navigation
{

GNSSPathPublisher::GNSSPathPublisher()
    :Node("gnss_path_publisher"),
    init_flag_(true)
{
    declareParameter();
    initCommunication();
    loadCSV();
    path_msg_ = setMsg(xs_, ys_);
    origin_path_msg_ = setMsg(origin_xs_, origin_ys_);    
}

GNSSPathPublisher::~GNSSPathPublisher() {}

// パラメータ宣言
void GNSSPathPublisher::declareParameter()
{
    this->declare_parameter("loop_freq", 1);
    this->declare_parameter<std::string>("file_path", "/home/nvidia/formula_ws/src/gnss_navigation/config/course_data/shihou_full_teleop.csv");
}

// pub, パラメータ設定
void GNSSPathPublisher::initCommunication(void)
{
    publisher_ = this->create_publisher<nav_msgs::msg::Path>("gnss_path", 10);
    origin_publisher_ = this->create_publisher<nav_msgs::msg::Path>("origin_gnss_path", 10);

    this->get_parameter("loop_freq", freq_);
    file_path_ = this->get_parameter("file_path").as_string();
}

// 初期位置を取得
void GNSSPathPublisher::setInitPose(double x, double y)
{
    base_x_ = x;
    base_y_ = y;
    init_flag_ = false;
}

// CSVファイルの読み込み
void GNSSPathPublisher::loadCSV(void)
{
    std::ifstream file(file_path_);

    while(std::getline(file, line_)) {
        std::stringstream ss(line_);
        tokens_.clear();
        while (std::getline(ss, cell_, ',')) {
            tokens_.push_back(cell_);
        }
        double lat = std::stod(tokens_[0]);
        double lon = std::stod(tokens_[1]);
        auto [x, y] = convertGPStoUTM(lat, lon);

        if (init_flag_) setInitPose(x, y);

        xs_.push_back(x - base_x_);
        ys_.push_back(y - base_y_);
        origin_xs_.push_back(x);
        origin_ys_.push_back(y);
    }
}

// pathの生成
nav_msgs::msg::Path GNSSPathPublisher::setMsg(const std::vector<double>& xs, const std::vector<double>& ys)
{
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

// スプライン補間
std::vector<Eigen::Vector2d> GNSSPathPublisher::interpolateSpline(const std::vector<double>& xs, const std::vector<double>& ys, int num_points) 
{
    Eigen::Matrix<double, Eigen::Dynamic, 2> points(xs.size(), 2);
    for (size_t i = 0; i < xs.size(); ++i) {
        points(i, 0) = xs[i];
        points(i, 1) = ys[i];
    }

    auto spline = Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(points.transpose(), 2); // 2次のキュービックスプライン

    double step = 1.0 / (num_points - 1);
    for (int i = 0; i < num_points; ++i) {
        double u = i * step;
        Eigen::Vector2d pt = spline(u);
        result_.push_back(pt);
    }
    return result_;
}

// WGS84系からUTM座標系へ変換
std::pair<double, double> GNSSPathPublisher::convertGPStoUTM(double lat, double lon) {
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

void GNSSPathPublisher::loop(void)
{
    publisher_->publish(path_msg_);
    origin_publisher_->publish(origin_path_msg_);
}

} // namespace gnss_navigation

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gnss_navigation::GNSSPathPublisher>();
    rclcpp::Rate loop_rate(node->freq_);
    while (rclcpp::ok()) {
        node->loop();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}