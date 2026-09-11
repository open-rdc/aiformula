#include "FollowPath.hh"

#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/math/PID.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/msgs/twist.pb.h>
#include <ignition/msgs/int32.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/common/Console.hh>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/boolean.pb.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
#include <vector>
#include <string>
#include <utility>
#include <cmath>
#include <mutex>
#include <algorithm>
#include <limits>
#include <iostream>

namespace ignition
{
namespace gazebo
{
    class FollowPathPrivate
    {
        public:
            void setPath(const ignition::msgs::Pose_V &_msg);
            void FindModelEntities(EntityComponentManager &_ecm);
            void StopRobot();
            bool setInitidx(const ignition::math::Vector3d& currentPos);
            bool updateTargetIndex(const ignition::math::Vector3d& currentPos);
            void OnCsvSwitchResponse(const ignition::msgs::Boolean &_rep, bool _result);
            bool requestCsvSwitch(int csv_id);
            void checkTimeoutAndRetry();

            ignition::transport::Node node_;
            ignition::transport::Node::Publisher cmd_vel_pub_;

            Entity RobotEntity = kNullEntity;

            std::vector<Eigen::Vector2d> result_;
            size_t current_idx_{0};

            std::mutex path_mutex_;

            bool setInitidxflag = false;
            bool switching_csv_ = false;
            bool is_initialized_ = false;
            ignition::math::PID pid_;

            std::chrono::steady_clock::time_point last_request_time_;
            int retry_count_{0};
            const int MAX_RETRIES = 3;
            const std::chrono::seconds REQUEST_TIMEOUT{3};
            
            std::string ROBOT_MODEL_NAME = "human_robot";
            double reach_threshold_{1.3};
            double constant_linear_vel_{1.0};  
            double p{0.7};
            double i{0.0};
            double d{0.15};
    };

    FollowPath::FollowPath() : dataPtr(std::make_unique<FollowPathPrivate>())
    {
    }

    FollowPath::~FollowPath() = default;

    namespace
    {
        double calculateTheta(const Eigen::Vector2d& target, 
                              const ignition::math::Vector3d& currentPos, 
                              double current_yaw)
        {
            double dx = target.x() - currentPos.X();
            double dy = target.y() - currentPos.Y();

            double target_angle = std::atan2(dy, dx);
            current_yaw = std::atan2(std::sin(current_yaw), std::cos(current_yaw));
            double theta = target_angle - current_yaw;

            return std::atan2(std::sin(theta), std::cos(theta)); 
        }
    }

    void FollowPath::Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, 
                              EntityComponentManager &_ecm, EventManager &_eventMgr)
    {
        std::string topic = "/gnss_path";
        if (_sdf && _sdf->HasElement("topic_name")) {
            topic = _sdf->Get<std::string>("topic_name");
        }

        if (!dataPtr->node_.Subscribe(topic, &FollowPathPrivate::setPath, dataPtr.get())) {
            std::cerr << "Failed to subscribe to topic [" << topic << "]" << std::endl;
        }

        std::string cmd_vel_topic = "/cmd_vel_obstacle";
        if (_sdf && _sdf->HasElement("cmd_vel_topic")) {
            cmd_vel_topic = _sdf->Get<std::string>("cmd_vel_topic");
        }

        dataPtr->cmd_vel_pub_ = dataPtr->node_.Advertise<ignition::msgs::Twist>(cmd_vel_topic);
        
        if (_sdf)
        {
            dataPtr->ROBOT_MODEL_NAME = (_sdf->HasElement("robot_name")) ? _sdf->Get<std::string>("robot_name") : "human_robot";
            dataPtr->reach_threshold_ = (_sdf->HasElement("threshold")) ? _sdf->Get<double>("threshold") : 1.3;
            dataPtr->constant_linear_vel_ = (_sdf->HasElement("linear_vel")) ? _sdf->Get<double>("linear_vel") : 1.0;
            dataPtr->p = (_sdf->HasElement("p_gain")) ? _sdf->Get<double>("p_gain") : 0.7;
            dataPtr->i = (_sdf->HasElement("i_gain")) ? _sdf->Get<double>("i_gain") : 0.0;
            dataPtr->d = (_sdf->HasElement("d_gain")) ? _sdf->Get<double>("d_gain") : 0.15;
        }

        dataPtr->pid_ = ignition::math::PID(dataPtr->p, dataPtr->i, dataPtr->d);
    }

    void FollowPathPrivate::setPath(const ignition::msgs::Pose_V &_msg)
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        
        result_.clear();
        for (int i = 0; i < _msg.pose_size(); ++i)
        {
            const auto &pose = _msg.pose(i);

            if (pose.has_position())
            {
                double px = pose.position().x();
                double py = pose.position().y();
                result_.emplace_back(px, py);
            } 
            else
            {
                std::cerr << "Pose at index " << i << " does not have position!" << std::endl;
            }
        }

        if (!result_.empty())
        {
            switching_csv_ = false;
            setInitidxflag = false;
            retry_count_ = 0;
            std::cout << "Received new path (" << result_.size() << " points)." << std::endl;
        }
    }

    void FollowPathPrivate::FindModelEntities(EntityComponentManager &_ecm)
    {
        _ecm.Each<components::Model, components::Name>(
            [&](const Entity &_entity,
                const components::Model *,
                const components::Name *_name) -> bool
            {
                if (_name->Data() == ROBOT_MODEL_NAME)
                {
                    RobotEntity = _entity;
                    return false;
                }
                return true;
            });
    }

    void FollowPathPrivate::StopRobot()
    {
        ignition::msgs::Twist msg;
        msg.mutable_linear()->set_x(0.0);
        msg.mutable_angular()->set_z(0.0);
        cmd_vel_pub_.Publish(msg);
    }

    bool FollowPathPrivate::setInitidx(const ignition::math::Vector3d& currentPos)
    {
        if (result_.empty())
        {
            return false;
        }

        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = 0;
        for (size_t i = 0; i < result_.size(); ++i)
        {
            const auto& target_pt = result_[i];
            double dx = target_pt.x() - currentPos.X();
            double dy = target_pt.y() - currentPos.Y();
            double dist = std::hypot(dx, dy);
            if (dist < min_dist)
            {
                closest_idx = i;
                min_dist = dist;
            }
        }

        current_idx_ = (closest_idx + 2) % result_.size();
        std::cout << "Initindex: " << current_idx_ << std::endl;
        return true;
    }

    bool FollowPathPrivate::updateTargetIndex(const ignition::math::Vector3d& currentPos)
    {
        if (result_.empty())
        {
            StopRobot();
            return false;
        }

        const auto& target_pt = result_[current_idx_];
        double dx = target_pt.x() - currentPos.X();
        double dy = target_pt.y() - currentPos.Y();
        double dist = std::hypot(dx, dy);

        if (dist < reach_threshold_ && current_idx_ < result_.size() - 1)
        {
            current_idx_++;
        }

        if (current_idx_ == result_.size() - 1 && dist < 0.5)
        {
            StopRobot();
            current_idx_++;
            return false;
        }

        return true;
    }

    void FollowPathPrivate::OnCsvSwitchResponse(const ignition::msgs::Boolean &_rep, bool _result)
    {

        if (_result && _rep.data())
        {
            std::cout << "CSV changed" << std::endl;
            retry_count_ = 0;
        }
        else
        {
            std::cout << "CSV not changed" << std::endl;
            switching_csv_ = false;
            setInitidxflag = false;
        }
    }

    bool FollowPathPrivate::requestCsvSwitch(int csv_id)
    {
        if (switching_csv_) return false;

        ignition::msgs::Int32 req;
        req.set_data(csv_id);

        std::string service_name = "/csv_switch";
        switching_csv_ = true;
        last_request_time_ = std::chrono::steady_clock::now();

        bool executed = node_.Request(
            service_name,
            req,
            &FollowPathPrivate::OnCsvSwitchResponse,
            this
        );

        if (!executed)
        {
            std::cout << "Service [" << service_name << "] failed to send" << std::endl;
            switching_csv_ = false;
            return false;
        }

        std::cout << "Service [" << service_name << "] successfully sent" << std::endl;
        return true;
    }

    void FollowPathPrivate::checkTimeoutAndRetry()
    {
        if (!switching_csv_) return;

        auto now = std::chrono::steady_clock::now();
        if (now - last_request_time_ > REQUEST_TIMEOUT)
        {
            retry_count_++;
            std::cout << "Timeout (" << retry_count_ << "/" << MAX_RETRIES << ")" << std::endl;

            switching_csv_ = false;

            if (retry_count_ >= MAX_RETRIES)
            {
                std::cerr << "Error: Reached maximum retry count." << std::endl;
                is_initialized_ = false;
                retry_count_ = 0;
                return;
            }

            requestCsvSwitch(rand() % 2 + 1);
        }
    }

    void FollowPath::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
    {
        if (_info.paused) return;
        std::lock_guard<std::mutex> lock(dataPtr->path_mutex_);

        if (dataPtr->RobotEntity == kNullEntity)
        {
            dataPtr->FindModelEntities(_ecm);
            if (dataPtr->RobotEntity == kNullEntity) return;
        }

        // timeout & retry check
        dataPtr->checkTimeoutAndRetry();

        // changing path, stop robot
        if (dataPtr->switching_csv_)
        {
            dataPtr->StopRobot();
            return;
        }

        // first startup, send csv switch request
        if (!dataPtr->is_initialized_)
        {
            std::cout << "First startup: Sending the first CSV switch request." << std::endl;
            if (dataPtr->requestCsvSwitch(rand() % 2 + 1))
            {
                dataPtr->is_initialized_ = true;
            }
            return;
        }

        if (dataPtr->result_.empty())
        {
            dataPtr->StopRobot();
            return;
        }

        auto poseComp = _ecm.Component<components::Pose>(dataPtr->RobotEntity);
        if (!poseComp) return;

        ignition::math::Vector3d currentPos = poseComp->Data().Pos();
        ignition::math::Quaterniond currentRot = poseComp->Data().Rot();

        double current_yaw = currentRot.Yaw();

        if (!dataPtr->setInitidxflag)
        {
            if (!dataPtr->setInitidx(currentPos))
            {
                return;
            }
            dataPtr->setInitidxflag = true;
        }

        if (!dataPtr->updateTargetIndex(currentPos)) return;

        if (dataPtr->current_idx_ >= dataPtr->result_.size())
        {
            std::cout << "Goal reached: sending CSV switch request" << std::endl;
            dataPtr->StopRobot();
            dataPtr->requestCsvSwitch(rand() % 2 + 1);
            return;
        }

        double theta = calculateTheta(dataPtr->result_[dataPtr->current_idx_], currentPos, current_yaw);

        std::chrono::duration<double> dt_duration = _info.dt;
        if (dt_duration.count() <= 0.0) return;

        double angular_vel = (-1) * dataPtr->pid_.Update(theta, dt_duration);

        ignition::msgs::Twist msg;
        msg.mutable_linear()->set_x(dataPtr->constant_linear_vel_); 
        msg.mutable_angular()->set_z(angular_vel);         

        dataPtr->cmd_vel_pub_.Publish(msg);
    }

} // namespace gazebo
} // namespace ignition

IGNITION_ADD_PLUGIN(
    ignition::gazebo::FollowPath,
    ignition::gazebo::System,
    ignition::gazebo::FollowPath::ISystemConfigure,
    ignition::gazebo::FollowPath::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::FollowPath, "ignition::gazebo::FollowPath")