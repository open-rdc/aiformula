#ifndef OBSTACLE_CONTROL_PLUGIN_HH_
#define OBSTACLE_CONTROL_PLUGIN_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Name.hh>
#include <nav_msgs/msg/path.hpp>
#include <ignition/transport/Node.hh>
#include <ignition/math/PID.hh>



#include <vector>
namespace ignition
{
namespace gazebo
{
    class PathPablisher : public System, public ISystemConfigure, public ISystemPreUpdate
    {
        public:

            PathPablisher();

            PathPablisher(const Entity &_emtity, const std::shared_ptr<const sdf::Element> &_sdf);

            void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, 
                EntityComponentManager &_ecm, EventManager &_eventMgr) override;

            void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override;
            
        
        private:
            void FindObstacleEntities(EntityComponentManager &_ecm);
            void LoadCSV();
            void setInitialPosition(double x, double y);
            void setPose(const std::vector<Entity>& entities, );
            transport::Node node;
            


            nav_msgs::msg::Path setPathMsg(const std::vector<double> &_xs, const std::vector<double> &_ys);
            std::vector<Eigen::Vector2d> interpolateSpline(const std::vector<double> &_xs, 
                const std::vector<double> &_ys, int num_points);
            std::pair<double, double> convertGPStoUTM(double lat, double lon)


            double x,y;
            std::vector<double> xs, ys;
            double lat, lon;

            std::vector <Entity> ObstacleEntities;

            std::string ROBOT_NAME;
    };

    class Follow : public System, public ISystemConfigure, public ISystemPreUpdate
    {
        public:
            Follow() = default;

            void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, 
                EntityComponentManager &_ecm, EventManager &_eventMgr) override;

            void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override;



    }
}
}