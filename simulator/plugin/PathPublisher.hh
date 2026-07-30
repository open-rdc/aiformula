#ifndef PATHPUBLISHER_PLUGIN_HH_
#define PATHPUBLISHER_PLUGIN_HH_

#include <ignition/gazebo/System.hh>
#include <memory>


namespace ignition
{
namespace gazebo
{
    class PathPublisherPrivate;

    class PathPublisher : public System, public ISystemConfigure, public ISystemPreUpdate
    {
        public:

            PathPublisher();
            ~PathPublisher() override;

            void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, 
                EntityComponentManager &_ecm, EventManager &_eventMgr) override;

            void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;
            
        
        private:
            std::unique_ptr<PathPublisherPrivate> dataPtr;
    };
}
}

#endif