#include "PathPublisher.hh"

#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/msgs/int32.pb.h>
#include <ignition/msgs/boolean.pb.h>

#include <unsupported/Eigen/Splines>
#include <Eigen/Dense>
#include <chrono>
#include <vector>
#include <string>
#include <utility>
#include <proj.h>
#include <fstream>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace ignition
{
namespace gazebo
{
    class PathPublisherPrivate
    {
        public:
            bool setcsv(const ignition::msgs::Int32 &_req, ignition::msgs::Boolean &_rep);
            void LoadCSV(const std::string file_path);
            void setInitPose(double x, double y);
            ignition::msgs::Pose_V setMsg(const std::vector<double>& xs, const std::vector<double>& ys);
            std::vector<Eigen::Vector2d> interpolateSpline(const std::vector<double>& xs, const std::vector<double>& ys, int num_points);
            std::pair<double, double> convertGPStoUTM(double lon, double lat);

            //std::string file_name_;
            std::string first_file_name_;
            std::string second_file_name_;
            std::string file_path_;
            std::string line_;
            std::string cell_;
            std::vector<std::string> token_;

            ignition::transport::Node node_;
            ignition::transport::Node::Publisher path_pub_;
            ignition::transport::Node::Publisher origin_path_pub_;

            ignition::msgs::Pose_V path_msg_;
            ignition::msgs::Pose_V origin_path_msg_;
            double base_x_{0.0};
            double base_y_{0.0};
            std::vector<double> xs_, ys_;
            std::vector<double> origin_xs_, origin_ys_;
            std::vector<Eigen::Vector2d> result_;

            bool init_flag_{true};
            bool path_update_flag_{true};
            std::string pre_file_name_;
            int cnt_{0};
            
            double step;
            std::chrono::steady_clock::duration last_update_time_{0};
    };

    PathPublisher::PathPublisher() : dataPtr(std::make_unique<PathPublisherPrivate>())
    {
    }

    PathPublisher::~PathPublisher() = default;

    void PathPublisher::Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, 
                    EntityComponentManager &_ecm, EventManager &_eventMgr)
    {
        dataPtr->first_file_name_ = (_sdf && _sdf->HasElement("first_file_name")) ? _sdf->Get<std::string>("first_file_name") : "dynamic_obstacle_left1";
        dataPtr->second_file_name_ = (_sdf && _sdf->HasElement("second_file_name")) ? _sdf->Get<std::string>("second_file_name") : "dynamic_obstacle_right1";
        
        dataPtr->path_pub_ = dataPtr->node_.Advertise<ignition::msgs::Pose_V>("/gnss_path");
        dataPtr->origin_path_pub_ = dataPtr->node_.Advertise<ignition::msgs::Pose_V>("/origin_gnss_path");

        std::string service_name = "/csv_switch";
        if (!dataPtr->node_.Advertise(service_name, &PathPublisherPrivate::setcsv, dataPtr.get())) {
            std::cerr << "Failed to advertise service [" << service_name << "]" << std::endl;
        }
    }


    bool PathPublisherPrivate::setcsv(const ignition::msgs::Int32 &_req, ignition::msgs::Boolean &_rep)
    {
        int selected_file = _req.data();
        std::string target_file_name;
        target_file_name = (selected_file == 1) ? first_file_name_ : second_file_name_;

        std::cout << "Selected CSV file: " << target_file_name << std::endl;

        if (target_file_name == pre_file_name_)
        {
            std::cout << "No changed CSV file: " << target_file_name << std::endl;
            _rep.set_data(false);
            return true;
        }
        else
        {
            pre_file_name_ = target_file_name;
            _rep.set_data(true);
            std::cout << "Changed CSV file: " << pre_file_name_ << std::endl;
        }

        std::string pkg_share = ament_index_cpp::get_package_share_directory("simulator");
        std::string file_path = pkg_share + "/plugin/config/" + target_file_name + ".csv";
        xs_.clear();
        ys_.clear();
        origin_xs_.clear();
        origin_ys_.clear();
        init_flag_ = true;

        LoadCSV(file_path);
        path_msg_ = setMsg(xs_, ys_);
        origin_path_msg_ = setMsg(origin_xs_, origin_ys_);

        path_pub_.Publish(path_msg_);
        origin_path_pub_.Publish(origin_path_msg_);

        return true;
    }

    void PathPublisherPrivate::LoadCSV(const std::string file_path)
    {
        std::ifstream file(file_path);
        printf("loadCSV: file_path = %s\n", file_path.c_str());

        if (!file.is_open())
        {
            std::cerr << "Failed to open csv file" << file_path << std::endl;
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


    void PathPublisherPrivate::setInitPose(double x, double y)
    {
        base_x_ = x;
        base_y_ = y;
        init_flag_ = false;
    }

    ignition::msgs::Pose_V PathPublisherPrivate::setMsg(const std::vector<double>& xs, const std::vector<double>& ys)
    {
        std::vector<Eigen::Vector2d> spline_points = interpolateSpline(xs, ys, 100);

        ignition::msgs::Pose_V path_msg;

        auto *data = path_msg.mutable_header()->add_data();
        data->set_key("frame_id");
        data->add_value("base_link");

        for (const auto& coord : spline_points) {
            auto *pose = path_msg.add_pose();
            pose->mutable_position()->set_x(coord.x());
            pose->mutable_position()->set_y(coord.y());
            pose->mutable_position()->set_z(0.0);
        }

        return path_msg;
    }

    std::vector<Eigen::Vector2d> PathPublisherPrivate::interpolateSpline(const std::vector<double>& xs, const std::vector<double>& ys, int num_points)
    {
        Eigen::Matrix<double, Eigen::Dynamic, 2> points(xs.size(), 2);
        result_.clear();
        for (size_t i=0; i < xs.size(); ++i)
        {
            points(i, 0) = xs[i];
            points(i, 1) = ys[i];
        }

        auto spline = Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(points.transpose(), 2);

        if(num_points > 1)
            step = 1.0 / (num_points -1);
        for (int i = 0; i < num_points; ++i)
        {
            double u = i * step;
            Eigen::Vector2d pt = spline(u);
            result_.push_back(pt);
        }
        return result_;
    }

    std::pair<double, double> PathPublisherPrivate::convertGPStoUTM(double lon, double lat)
    {
        if (!(-90 <= lat) || !(lat <= 90) || !(-180 <= lon) || !(lon <= 180))
        {
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
    }
}
}

IGNITION_ADD_PLUGIN(
    ignition::gazebo::PathPublisher,
    ignition::gazebo::System,
    ignition::gazebo::PathPublisher::ISystemConfigure,
    ignition::gazebo::PathPublisher::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::PathPublisher, "ignition::gazebo::PathPublisher")
