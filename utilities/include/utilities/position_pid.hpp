#pragma once

namespace controller{

class PositionPid{
public:
    PositionPid(const int sampling_time_ms): sampling_time(static_cast<double>(sampling_time_ms)/1000.0){}
    double cycle(double current_pos, double target_pos){
        double error = target_pos - current_pos;
        return cycle(error);
    }
    double cycle(double error){
        error_ = error;
        integral += error * sampling_time;
        const double operation = error*kp_ + integral*ki_ + (error-old_error)/sampling_time*kd_;
        old_error = error;
        return operation;
    }
    void gain(const double kp, const double ki, const double kd){
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }
    void reset(){
        integral = 0.0;
        old_error = 0.0;
    };

    double kp(){ return kp_;}
    double ki(){ return ki_;}
    double kd(){ return kd_;}
    double error(){ return error_;}

private:
    const double sampling_time;

    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;
    double error_ = 0.0;

    double integral = 0.0;
    double old_error = 0.0;
};
}   // namespace controller
