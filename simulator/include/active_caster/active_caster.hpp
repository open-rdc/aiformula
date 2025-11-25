#pragma once
#include <std_msgs/msg/float64.hpp>
#include "base/velplanner.hpp"
#include "base/position_pid.hpp"


namespace chassis_driver{
class ChassisDriver : public rclpp::Node{
	public:
		CHASSIS_DRIVER_PUBLIC
                explicit ChassisDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

                CHASSIS_DRIVER_PUBLIC
                explicit ChassisDriver(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());


	private:
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscription_vel;
		void _subscriber_callback_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
		rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr publisher_odrive;
		rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr odrive_axis_client_;
		rclcpp::QoS _qos = rclcpp::QoS(10);

		// 速度計画機
    		velplanner::VelPlanner linear_planner;
    		const velplanner::Limit linear_limit;
    		velplanner::VelPlanner angular_planner;
    		const velplanner::Limit angular_limit;

    		// 従動輪のPID
    		controller::PositionPid caster_pid;


		// 定数
    		const int interval_ms;
    		const double wheel_radius;
    		const double tread;
    		const double wheelbase;
    		const double rotate_ratio;
    		const bool is_reverse_left;
    		const bool is_reverse_right;
    		const int caster_max_count;
    		const double caster_max_angle;
    		const double motor_max_torque;

    		// 変数
    		double caster_orientation = 0.0;

    		// 動作モード
    		enum class Mode{
        		cmd,
        		stay,
        		stop
    		} mode = Mode::stay;

};
}
