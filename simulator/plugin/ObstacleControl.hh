#ifndef OBSTACLE_CONTROL_PLUGIN_HH_
#define OBSTACLE_CONTROL_PLUGIN_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/transport/Node.hh>
#include <ignition/math/PID.hh>
#include <ignition/msgs/pose_v.pb.h>

#include <Eigen/Dense>
#include <chrono>
#include <vector>
#include <string>
#include <utility>
#include <memory>



#include <vector>
namespace ignition
{
namespace gazebo
{
    class PathPublisher : public System, public ISystemConfigure, public ISystemPreUpdate
    {
        public:

            PathPublisher() = default;
            ~PathPublisher() override = default;

            void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, 
                EntityComponentManager &_ecm, EventManager &_eventMgr) override;

            void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;
            
        
        private:
            void LoadCSV();
            void setInitPose(double x, double y);
            ignition::msgs::Pose_V setMsg(const std::vector<double>& xs, const std::vector<double>& ys);
            std::vector<Eigen::Vector2d> interpolateSpline(const std::vector<double>& xs, const std::vector<double>& ys, int num_points);
            std::pair<double, double> convertGPStoUTM(double lon, double lat);

            std::string file_path_;
            ignition::transport::Node node_;
            ignition::transport::Node::Publisher path_pub_;
            ignition::transport::Node::Publisher origin_path_pub_;
            ignition::msgs::Pose_V path_msg_;
            ignition::msgs::Pose_V origin_path_msg_;

            std::string line_;
            std::string cell_;
            std::vector<std::string> token_;
            bool init_flag_{true};
            double base_x_{0.0};
            double base_y_{0.0};

            std::vector<double> xs_, ys_;
            std::vector<double> origin_xs_, origin_ys_;
            std::vector<Eigen::Vector2d> result_;
            double step;
            std::string file_name;

            std::chrono::steady_clock::duration last_update_time_{0};
    };
}
}

#endif