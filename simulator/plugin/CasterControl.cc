#include "CasterControl.hh"

#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/transport/Node.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/plugin/Register.hh>

#include <cmath>
#include <mutex>
#include <iostream>

namespace ignition
{
namespace gazebo
{
    class CasterControlPrivate
    {
      public: 
        void JointForceCmdCallback(const ignition::msgs::Double &_msg);
        void FindJointEntities(EntityComponentManager &_ecm);
        void setBase();
        
        ignition::transport::Node node_;
        ignition::transport::Node::Publisher jointOrientationPub_;
        
        //std::string MODEL_NAME;
        std::string PULLEY_JOINT_NAME;
        std::string ROTATOR_JOINT_NAME;

        double pulley_radius{0.015};
        double rotator_radius{0.035};
        double spring_length{0.0};
        double alpha{0.174}; // 不感帯
        double K{500.0};     // バネ定数
        double Damp{0.02};
        double pulley_theta{0.0};
        double rotator_theta{0.0};
        bool isBaseSet{false};
        double x{0.0};
        double ReactionForce{0.0};

        std::mutex mutex_;
        ignition::gazebo::Model model_;
        std::string SubtopicName;
        std::string PubtopicName;
        Entity modelEntity = kNullEntity;
        Entity PulleyJointEntity = kNullEntity;
        Entity RotatorJointEntity = kNullEntity;
    };

    CasterControl::CasterControl() : dataPtr(std::make_unique<CasterControlPrivate>())
    {
    }

    CasterControl::~CasterControl() = default;

    void CasterControlPrivate::JointForceCmdCallback(const ignition::msgs::Double &_msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pulley_theta = _msg.data();
    }

    void CasterControlPrivate::FindJointEntities(EntityComponentManager &_ecm)
    {
        _ecm.Each<components::Joint, components::Name>(
            [&](const Entity &_entity,
                const components::Joint * /*_joint*/,
                const components::Name *_name) -> bool
            {
                if (_name->Data() == PULLEY_JOINT_NAME)
                {
                    std::cout << "Found Pulley"<< std::endl;
                    PulleyJointEntity = _entity;
                }
                else if (_name->Data() == ROTATOR_JOINT_NAME)
                {
                    std::cout << "Found Rotator"<< std::endl;
                    RotatorJointEntity = _entity;
                }
                if (PulleyJointEntity != kNullEntity && RotatorJointEntity != kNullEntity)
                {
                    return false;
                }
                return true;
            });
    }

    void CasterControlPrivate::setBase()
    {
        x = 0.0;
        ReactionForce = 0.0;
        pulley_theta = 0.0;
        rotator_theta = 0.0;
        isBaseSet = true;
    }

    void CasterControl::Configure(const Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        EntityComponentManager &_ecm,
        EventManager &/*_eventMgr*/)
    {
        dataPtr->model_ = Model(_entity);
        if (dataPtr->model_.Valid(_ecm))
        {
            dataPtr->modelEntity = _entity;
        }
        else
        {
            std::cerr << "CasterControl plugin should be attached to a model entity.\n" 
                      << "Plugin will not function correctly." << std::endl;
            return;
        }

        if (_sdf)
        {
            dataPtr->PULLEY_JOINT_NAME = (_sdf->HasElement("pulley_joint") ? _sdf->Get<std::string>("pulley_joint") : "caster_reel_joint");
            dataPtr->ROTATOR_JOINT_NAME = (_sdf->HasElement("rotator_joint") ? _sdf->Get<std::string>("rotator_joint") : "caster_steer_joint");
            dataPtr->SubtopicName = (_sdf->HasElement("sub_topic_name") ? _sdf->Get<std::string>("sub_topic_name") : "/caster_reel_position_cmd");
            dataPtr->PubtopicName = (_sdf->HasElement("pub_topic_name") ? _sdf->Get<std::string>("pub_topic_name") : "/caster_orientation");
            dataPtr->Damp = (_sdf->HasElement("damping") ? _sdf->Get<double>("damping") : 0.02);
            dataPtr->K = (_sdf->HasElement("k_spring") ? _sdf->Get<double>("k_spring") : 500.0);
            dataPtr->alpha = (_sdf->HasElement("alpha") ? _sdf->Get<double>("alpha") : 0.174);
            dataPtr->pulley_radius = (_sdf->HasElement("pulley_radius") ? _sdf->Get<double>("pulley_radius") : 0.015);
            dataPtr->rotator_radius = (_sdf->HasElement("rotator_radius") ? _sdf->Get<double>("rotator_radius") : 0.035);
        }

        dataPtr->node_.Subscribe(dataPtr->SubtopicName, &CasterControlPrivate::JointForceCmdCallback, dataPtr.get());
        dataPtr->jointOrientationPub_ = dataPtr->node_.Advertise<ignition::msgs::Double>(dataPtr->PubtopicName);

        dataPtr->setBase();
        std::cout << "CasterControl Plugin configured successfully.\n" << std::endl;
        std::cout << "Pulley Joint Name: " << dataPtr->PULLEY_JOINT_NAME << std::endl;
        std::cout << "Rotator Joint Name: " << dataPtr->ROTATOR_JOINT_NAME << std::endl;
        std::cout << "Subtopic Name: " << dataPtr->SubtopicName << std::endl;
        std::cout << "Pubtopic Name: " << dataPtr->PubtopicName << std::endl;
        std::cout << "Damping: " << dataPtr->Damp << std::endl;
        std::cout << "Spring Constant (K): " << dataPtr->K << std::endl;
        std::cout << "Alpha: " << dataPtr->alpha << std::endl;
        std::cout << "Pulley Radius: " << dataPtr->pulley_radius << std::endl;
        std::cout << "Rotator Radius: " << dataPtr->rotator_radius << std::endl;
    }

    void CasterControl::PreUpdate(const UpdateInfo &_info,
        EntityComponentManager &_ecm)
    {
        if (_info.paused) return;
        // ジョイントEntityの初回探索
        if (dataPtr->RotatorJointEntity == kNullEntity)
        {
            dataPtr->FindJointEntities(_ecm);
            if (dataPtr->RotatorJointEntity == kNullEntity) return;
        }
        // 関節位置 (rotator_theta) の取得
        auto posComp = _ecm.Component<components::JointPosition>(dataPtr->RotatorJointEntity);
        std::cout << "RotatorJointEntity: " << dataPtr->RotatorJointEntity << std::endl;
        std::cout << "Rotator Joint Position: " << (posComp ? std::to_string(posComp->Data()[0]) : "Component not found") << std::endl;
        if (!posComp)
        {
            _ecm.CreateComponent(this->dataPtr->RotatorJointEntity,
                components::JointPosition());
        }
        if (!posComp || posComp->Data().empty()) return;
        double raw_theta = posComp->Data()[0];

        // 正規化処理
        dataPtr->rotator_theta = std::atan2(std::sin(raw_theta), std::cos(raw_theta));
        std::cout << "rotator_theta (relative to parent): " << dataPtr->rotator_theta << std::endl;
            
        double current_pulley_theta = 0.0;
        {
            std::lock_guard<std::mutex> lock(dataPtr->mutex_);
            current_pulley_theta = dataPtr->pulley_theta;
            std::cout << "current_pulley_theta: " << current_pulley_theta << std::endl;
        }

        // カム機構の計算
        double abs_rotator_theta = std::abs(dataPtr->rotator_theta);
        int sign = (dataPtr->rotator_theta > 0) - (dataPtr->rotator_theta < 0);
        

        if (abs_rotator_theta < dataPtr->alpha)
        {
            dataPtr->x = dataPtr->rotator_radius * std::sin(abs_rotator_theta);
        }
        else
        {
            dataPtr->x = dataPtr->rotator_radius * std::sin(dataPtr->alpha) + 
                         dataPtr->rotator_radius * std::sin(abs_rotator_theta - dataPtr->alpha);
        }

        // バネ力と法線反力の計算
        double preload = dataPtr->pulley_radius * current_pulley_theta;
        std::cout << "preload: " << preload << std::endl;
        double spring_force = dataPtr->K * (dataPtr->x + preload);

        double F_r = 0.0;
        if (abs_rotator_theta < dataPtr->alpha)
        {
            F_r = spring_force * std::cos(abs_rotator_theta);
        }
        else
        {
            F_r = spring_force * std::cos(abs_rotator_theta - dataPtr->alpha);
        }

        // 復元トルク
        dataPtr->ReactionForce = -sign * dataPtr->rotator_radius * F_r;
        std::cout << "ReactionForce: " << dataPtr->ReactionForce << std::endl;

        auto forceCmdComp = _ecm.Component<components::JointForceCmd>(dataPtr->RotatorJointEntity);
        if (!forceCmdComp)
        {
            _ecm.CreateComponent(dataPtr->RotatorJointEntity, components::JointForceCmd({dataPtr->ReactionForce}));
        }
        else
        {
            forceCmdComp->Data()[0] = dataPtr->ReactionForce;
        }
    }

    void CasterControl::PostUpdate(const UpdateInfo &_info,
        const EntityComponentManager &/*_ecm*/)
    {
        if (_info.paused) return;

        ignition::msgs::Double msg;
        msg.set_data(dataPtr->rotator_theta);
        dataPtr->jointOrientationPub_.Publish(msg);
    }

}  // namespace gazebo
}  // namespace ignition

IGNITION_ADD_PLUGIN(
    ignition::gazebo::CasterControl,
    ignition::gazebo::System,
    ignition::gazebo::CasterControl::ISystemConfigure,
    ignition::gazebo::CasterControl::ISystemPreUpdate,
    ignition::gazebo::CasterControl::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::CasterControl, "ignition::gazebo::CasterControl")