#pragma once

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <Eigen/Dense>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "frenet_planner/obstacle_detect.hpp"
#include "frenet_planner/reference_curve.hpp"
#include "frenet_planner/risk_calculator.hpp"

namespace frenet_planner {

class QuinticPolynomial {
public:
    QuinticPolynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double T) {
        a0 = xs;
        a1 = vxs;
        a2 = axs / 2.0;

        Eigen::Matrix3d A;
        A << T*T*T, T*T*T*T, T*T*T*T*T,
             3*T*T, 4*T*T*T, 5*T*T*T*T,
             6*T, 12*T*T, 20*T*T*T;

        Eigen::Vector3d b;
        b << xe - a0 - a1*T - a2*T*T,
             vxe - a1 - 2*a2*T,
             axe - 2*a2;

        Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);

        a3 = x(0);
        a4 = x(1);
        a5 = x(2);
    }

    inline double calc_point(double t) const {
        return a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t;
    }

    inline double calc_first_derivative(double t) const {
        return a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t + 5*a5*t*t*t*t;
    }

    inline double calc_second_derivative(double t) const {
        return 2*a2 + 6*a3*t + 12*a4*t*t + 20*a5*t*t*t;
    }

    inline double calc_third_derivative(double t) const {
        return 6*a3 + 24*a4*t + 60*a5*t*t;
    }

private:
    double a0, a1, a2, a3, a4, a5;
};

class QuarticPolynomial {
public:
    QuarticPolynomial(double xs, double vxs, double axs, double vxe, double axe, double T) {
        a0 = xs;
        a1 = vxs;
        a2 = axs / 2.0;

        Eigen::Matrix2d A;
        A << 3*T*T, 4*T*T*T,
             6*T,   12*T*T;

        Eigen::Vector2d b;
        b << vxe - a1 - 2*a2*T,
             axe - 2*a2;

        Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);

        a3 = x(0);
        a4 = x(1);
    }

    inline double calc_point(double t) const {
        return a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t;
    }

    inline double calc_first_derivative(double t) const {
        return a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t;
    }

    inline double calc_second_derivative(double t) const {
        return 2*a2 + 6*a3*t + 12*a4*t*t;
    }

    inline double calc_third_derivative(double t) const {
        return 6*a3 + 24*a4*t;
    }

private:
    double a0, a1, a2, a3, a4;
};

struct VehicleState {
    double s;
    double s_dot;
    double s_ddot;
    double d;
    double d_dot;
    double d_ddot;
};

struct TrajectoryPoint {
    double s;
    double d;
    double x;
    double y;
    double yaw;
    double curvature;
    double velocity;
    double acceleration;
};

struct FrenetState {
    double s;
    double s_dot;
    double s_ddot;
    double d;
    double d_s;
    double d_ss;
};

struct CartesianState {
    double x;
    double y;
    double yaw;
    double curvature;
    double velocity;
    double acceleration;
};

struct FrenetTrajectory {
    std::vector<TrajectoryPoint> points;
    std::vector<double> t;
    std::vector<double> s;
    std::vector<double> d;
    std::vector<double> d_s;
    std::vector<double> d_ss;
    std::vector<double> s_dot;
    std::vector<double> d_dot;
    std::vector<double> s_ddot;
    std::vector<double> d_ddot;
    std::vector<double> s_dddot;
    std::vector<double> d_dddot;
    double cost;
    bool valid;
};

class FrenetPlanner {
public:
    explicit FrenetPlanner(RiskCalculator& risk_calculator);

    void set_parameters(
        double max_speed,
        double max_accel,
        double max_curvature,
        double dt,
        double d_t_s,
        int n_s_sample,
        double target_speed,
        double safety_margin
    );

    nav_msgs::msg::Path plan_local_path(
        const nav_msgs::msg::Path::SharedPtr reference_path,
        const std::vector<Obstacle>& obstacles,
        const geometry_msgs::msg::Twist& current_twist
    );

private:
    double max_speed_;
    double max_accel_;
    double max_curvature_;
    const double max_road_width_left_ = 4.0;
    const double max_road_width_right_ = 4.0;
    const double d_road_width_ = 0.5;
    double dt_;
    double d_t_s_;
    int n_s_sample_;
    double target_speed_;
    const double robot_radius_ = 0.5;
    double safety_margin_;

    RiskCalculator& risk_calculator_;

    FrenetTrajectory generate_lateral_trajectory(
        const VehicleState& current_state,
        double target_d,
        const FrenetTrajectory& longitudinal_traj);

    FrenetTrajectory generate_longitudinal_trajectory(
        const VehicleState& current_state,
        double target_speed,
        double T);

    bool select_best_path(
        const VehicleState& current_state,
        const nav_msgs::msg::Path::SharedPtr& reference_path,
        const std::vector<Obstacle>& obstacles,
        FrenetTrajectory& best_path,
        size_t& total_count,
        size_t& valid_count
    );

    void get_frenet_state(
        const nav_msgs::msg::Path::SharedPtr& path,
        const geometry_msgs::msg::Twist& current_twist,
        VehicleState& state
    );

    void frenet_to_cartesian(
        const ReferenceCurve& reference_curve,
        const FrenetState& state,
        CartesianState& out
    );
};

}  // namespace frenet_planner
