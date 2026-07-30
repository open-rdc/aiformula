#ifndef FOLLOWPATH_PLUGIN_HH_
#define FOLLOWPATH_PLUGIN_HH_

#include <ignition/gazebo/System.hh>
#include <memory>

namespace ignition
{
namespace gazebo
{
    class FollowPathPrivate;

    class FollowPath : public System, public ISystemConfigure, public ISystemPreUpdate
    {
        public:

            FollowPath();
            ~FollowPath() override;

            void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, 
                EntityComponentManager &_ecm, EventManager &_eventMgr) override;

            void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override;
            
        
        private:
            std::unique_ptr<FollowPathPrivate> dataPtr;
    };
}
}

#endif
