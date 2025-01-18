#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <proj.h>

class VectorvavSubscriber : public rclcpp::Node{
public:
    explicit VectorvavSubscriber(): Node("vectornav_subscriber"){
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "vectornav/pose",
            10,
            std::bind(&VectorvavSubscriber::pose_callback, this, std::placeholders::_1));

        gnss_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "vectornav/gnss",
            10,
            std::bind(&VectorvavSubscriber::gnss_callback, this, std::placeholders::_1));

        ecef_proj = proj_create_crs_to_crs(PJ_DEFAULT_CTX,
            "EPSG:4978", // ECEF
            "EPSG:32654", // UTMゾーン54N
            nullptr);
        gps_proj = proj_create_crs_to_crs(PJ_DEFAULT_CTX,
            "EPSG:4326",    // GPS
            "EPSG:32654",   // UTMゾーン54N
            nullptr);

        RCLCPP_INFO(this->get_logger(), "VectorNavSubscriber node initialized.");
    }
    ~VectorvavSubscriber(){
        proj_destroy(ecef_proj);
        proj_destroy(gps_proj);
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
        auto [x, y] = convertECEFtoUTM(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "pose:[x: %.2f, y: %.2f]", x, y);
        RCLCPP_INFO(this->get_logger(), "--------------------------------");

    }

    void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg){
        auto [x, y] = convertGPStoUTM(msg->latitude, msg->longitude);
        RCLCPP_INFO(this->get_logger(), "gnss:[x: %.2f, y: %.2f]", x, y);
    }


    std::pair<double, double> convertECEFtoUTM(const double x, const double y, const double z){
        if(ecef_proj == nullptr) {
            std::cerr << "ECEF PROJ error!" << std::endl;
        }
        PJ_COORD a, b;

        a.xyz.x = x;
        a.xyz.y = y;
        a.xyz.z = z;

        b = proj_trans(ecef_proj, PJ_FWD, a);

        return {b.enu.e, b.enu.n};
    }
    std::pair<double, double> convertGPStoUTM(const double lat, const double lon) {
        if(gps_proj == nullptr) {
            std::cerr << "GPS PROJ error!" << std::endl;
        }
        if (!(-90.0 <= lat) || !(lat <= 90.0) || !(-180.0 <= lon) || !(lon <= 180.0)) {
            std::cerr << "Latitude or longitude range error!" << std::endl;
            return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
        }
        PJ_COORD p = proj_coord(lat, lon, 0, 0);
        p = proj_trans(gps_proj, PJ_FWD, p);
        return {p.xy.x, p.xy.y};
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscription_;

    PJ *gps_proj, *ecef_proj;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VectorvavSubscriber>());
    rclcpp::shutdown();
    return 0;
}
