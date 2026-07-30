#include "ObstacleControl.hh"

#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose_v.pb.h>

#include <proj.h>
#include <unsupported/Eigen/Splines>
#include <fstream>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace ignition;
using namespace gazebo;

void PathPublisher::Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, 
                EntityComponentManager &_ecm, EventManager &_eventMgr)
{
    if (_sdf && _sdf->HasElement("file_name")) {
        file_name = _sdf->Get<std::string>("file_name");
    } else {
        std::cerr << "Set csv file in sdf file" << std::endl;
    }
    std::string pkg_share = ament_index_cpp::get_package_share_directory("simulator");
        file_path_ = pkg_share + "/plugin/config/" + file_name + ".csv";

    path_pub_ = node_.Advertise<ignition::msgs::Pose_V>("/gnss_path");
    origin_path_pub_ = node_.Advertise<ignition::msgs::Pose_V>("/origin_gnss_path");

    LoadCSV();
    path_msg_ = setMsg(xs_, ys_);
    origin_path_msg_ = setMsg(origin_xs_, origin_ys_);
}


void PathPublisher::LoadCSV()
{
    std::ifstream file(file_path_);
    printf("loadCSV: file_path_ = %s\n", file_path_.c_str());

    if (!file.is_open())
    {
        std::cerr << "Failed to open csv file" << file_path_ << std::endl;
        return;
    }
    else
    {
        std::cout << "Success to open csv file" << std::endl;
    }

    while (std::getline(file, line_))
    {
        std::stringstream ss(line_);
        token_.clear();
        while (std::getline(ss, cell_, ','))
       {
            token_.push_back(cell_);
        }
        double lat = std::stod(token_[0]);
        double lon = std::stod(token_[1]);
        auto [x, y] = convertGPStoUTM(lon, lat);

        if (init_flag_) setInitPose(x, y);
        xs_.push_back(x - base_x_);
        ys_.push_back(y - base_y_);
        origin_xs_.push_back(x);
        origin_ys_.push_back(y);
    }
}


void PathPublisher::setInitPose(double x, double y)
{
    base_x_ = x;
    base_y_ = y;
    init_flag_ = false;
}

ignition::msgs::Pose_V PathPublisher::setMsg(const std::vector<double>& xs, const std::vector<double>& ys){
    std::vector<Eigen::Vector2d> spline_points = interpolateSpline(xs, ys, 100);

    ignition::msgs::Pose_V path_msg;

    // ★ 修正箇所: set_value() ではなく set_key() と add_value() を使う
    auto *data = path_msg.mutable_header()->add_data();
    data->set_key("frame_id");
    data->add_value("base_link"); // ※ RVizの Fixed Frame で指定する座標系名

    // 既存の処理
    for (const auto& coord : spline_points) {
        auto *pose = path_msg.add_pose();
        pose->mutable_position()->set_x(coord.x());
        pose->mutable_position()->set_y(coord.y());
        pose->mutable_position()->set_z(0.0);
    }

    return path_msg;
}

std::vector<Eigen::Vector2d> PathPublisher::interpolateSpline(const std::vector<double>& xs, const std::vector<double>& ys, int num_points){
    Eigen::Matrix<double, Eigen::Dynamic, 2> points(xs.size(), 2);
    result_.clear();
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

std::pair<double, double> PathPublisher::convertGPStoUTM(double lon, double lat) {
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

void PathPublisher::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
    if (_info.paused) return;

    if (_info.simTime - last_update_time_ >= std::chrono::milliseconds(100)) {
        last_update_time_ = _info.simTime;

        path_pub_.Publish(path_msg_);
        origin_path_pub_.Publish(origin_path_msg_);
    }
}
    

IGNITION_ADD_PLUGIN(
    PathPublisher,
    ignition::gazebo::System,
    PathPublisher::ISystemConfigure,
    PathPublisher::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(PathPublisher, "ignition::gazebo::PathPublisher")
