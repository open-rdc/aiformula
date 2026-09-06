#include "PathPublisher.hh"

#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/msgs/int32.pb.h>

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
            void setcsv(const ignition::msgs::Int32 &_msg);
            void LoadCSV();
            void setInitPose(double x, double y);
            ignition::msgs::Pose_V setMsg(const std::vector<double>& xs, const std::vector<double>& ys);
            std::vector<Eigen::Vector2d> interpolateSpline(const std::vector<double>& xs, const std::vector<double>& ys, int num_points);
            std::pair<double, double> convertGPStoUTM(double lon, double lat);

            std::string file_name;
            std::string first_file_name;
            std::string second_file_name;
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
        if (_sdf && _sdf->HasElement("first_file_name"))
        {
            dataPtr->first_file_name = _sdf->Get<std::string>("first_file_name");
        }
        else
        {
            std::cerr << "Please set first csv file in sdf file!!!!!!!!" << std::endl;
            return;
        }

        if (_sdf && _sdf->HasElement("second_file_name"))
        {
            dataPtr->second_file_name = _sdf->Get<std::string>("second_file_name");
        }
        else
        {
            std::cerr << "Please set second csv file in sdf file!!!!!!!!" << std::endl;
            return;
        }

        if (rand() % 2 + 1 == 1)
        {
            dataPtr->file_name = dataPtr->first_file_name;
        }
        else
        {
            dataPtr->file_name = dataPtr->second_file_name;
        }
        std::cout << "Selected CSV file: " << dataPtr->file_name << std::endl;
        
        std::string pkg_share = ament_index_cpp::get_package_share_directory("simulator");
        dataPtr->file_path_ = pkg_share + "/plugin/config/" + dataPtr->file_name + ".csv";

        dataPtr->path_pub_ = dataPtr->node_.Advertise<ignition::msgs::Pose_V>("/gnss_path");
        dataPtr->origin_path_pub_ = dataPtr->node_.Advertise<ignition::msgs::Pose_V>("/origin_gnss_path");

        dataPtr->LoadCSV();

        std::string topic = "/csv_switch";
        if (!dataPtr->node_.Subscribe(topic, &PathPublisherPrivate::setcsv, dataPtr.get())) {
            ignerr << "Failed to subscribe to topic [" << topic << "]" << std::endl;
        }

        dataPtr->path_msg_ = dataPtr->setMsg(dataPtr->xs_, dataPtr->ys_);
        dataPtr->origin_path_msg_ = dataPtr->setMsg(dataPtr->origin_xs_, dataPtr->origin_ys_);
    }


    void PathPublisherPrivate::setcsv(const ignition::msgs::Int32 &_msg)
    {
        int selected_file = _msg.data();
        if (selected_file == 1)
        {
            file_name = first_file_name;
        }
        else if (selected_file == 2)
        {
            file_name = second_file_name;
        }
        else
        {
            std::cerr << "Invalid message received: " << selected_file << std::endl;
            return;
        }
        std::cout << "Selected CSV file: " << file_name << std::endl;

        std::string pkg_share = ament_index_cpp::get_package_share_directory("simulator");
        file_path_ = pkg_share + "/plugin/config/" + file_name + ".csv";
        xs_.clear();
        ys_.clear();
        origin_xs_.clear();
        origin_ys_.clear();

        LoadCSV();
        path_msg_ = setMsg(xs_, ys_);
        origin_path_msg_ = setMsg(origin_xs_, origin_ys_);
    }

    void PathPublisherPrivate::LoadCSV()
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

        if (_info.simTime - dataPtr->last_update_time_ >= std::chrono::milliseconds(100)) {
            dataPtr->last_update_time_ = _info.simTime;

            dataPtr->path_pub_.Publish(dataPtr->path_msg_);
            dataPtr->origin_path_pub_.Publish(dataPtr->origin_path_msg_);
        }
    }
}
}
 

IGNITION_ADD_PLUGIN(
    ignition::gazebo::PathPublisher,
    ignition::gazebo::System,
    ignition::gazebo::PathPublisher::ISystemConfigure,
    ignition::gazebo::PathPublisher::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::PathPublisher, "ignition::gazebo::PathPublisher")
