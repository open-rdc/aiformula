#include "utilities/velplanner.hpp"
#include "utilities/utils.hpp"

using namespace utils;

namespace velplanner{

rclcpp::Clock system_clock(RCL_ROS_TIME);
int64_t micros(){
    return system_clock.now().nanoseconds()*1e-3;
}

//VelPlanner
void VelPlanner::cycle(){
    if (mode == Mode::vel){
        const double time = (micros() - start_time) / 1000000.0;
        if (time < t1){
            old_time = micros();
            current_.acc = using_acc;
            current_.vel = using_acc * time + first.vel;
            current_.pos = using_acc / 2.0 * time * time + first.vel * time + first.pos;
        }
        else{
            mode = Mode::uniform_acceleration;
            achieved_target = true;
            current_ = target;
            cycle();
        }
    }
    else if(mode == Mode::uniform_acceleration){
        const double dt = (micros() - old_time) / 1000000.0;
        old_time = micros();
        current_.pos += current_.acc / 2.0 * dt * dt + current_.vel * dt;
        current_.vel += current_.acc * dt;
    }
    else{
        current_.vel = 0.0;
        current_.acc = 0.0;
    }
}

void VelPlanner::current(const Physics_t physics){
    current_ = physics;
    mode = Mode::uniform_acceleration;
    old_time = micros();
}

void VelPlanner::vel(double vel){
    this->vel(vel, micros());
}

void VelPlanner::vel(double vel, double start_time){
    this->vel(vel, (int64_t)(start_time * 1000000));
}

void VelPlanner::vel(double vel, int64_t start_time_us){
    start_time = start_time_us;
    achieved_target = false;
    mode = Mode::vel;
    first = current_;
    target.acc = 0.0;
    target.vel = constrain(vel, -limit_.vel, limit_.vel);
    const double diff_vel = target.vel - first.vel;
    using_acc = (diff_vel >= 0) ? limit_.acc : -limit_.acc;
    t1 = diff_vel / using_acc;
    target.pos = using_acc * t1 * t1 / 2.0 + first.vel * t1 + first.pos;
}

}
