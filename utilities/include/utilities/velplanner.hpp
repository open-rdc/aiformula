#pragma once

#include <stdint.h>
#include <rclcpp/rclcpp.hpp>

namespace velplanner{

struct Physics_t{
    Physics_t(){}
    Physics_t(double pos, double vel, double acc):pos(pos), vel(vel), acc(acc){}
    double pos = 0.0;
    double vel = 0.0;
    double acc = 0.0;
};

class VelPlanner{
public:
    VelPlanner(const Physics_t limit): limit_(limit){}
    void cycle();
    void current(const Physics_t physics);
    void limit(const Physics_t limit){ limit_ = limit; }

    void vel(double vel);
    void vel(double vel, double start_time);
    void vel(double vel, int64_t start_time_us);

    const double pos(){ return current_.pos; }
    const double vel(){ return current_.vel; }
    const double acc(){ return current_.acc; }

    const Physics_t current(){ return current_; }
    const bool hasAchievedTarget(){ return achieved_target; }

private:
    Physics_t limit_, first, target, current_;

    int64_t start_time = 0;
    int64_t old_time = 0;

    double t1 = 0.0;
    double using_acc = 0.0;

    enum class Mode{
        vel,
        uniform_acceleration
    } mode = Mode::uniform_acceleration;

    bool achieved_target = false;
};

//alias
using Limit = Physics_t;

}
