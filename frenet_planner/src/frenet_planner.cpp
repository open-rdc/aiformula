#include "frenet_planner/frenet_planner.hpp"
#include <numeric>
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

namespace frenet_planner {

FrenetPlanner::FrenetPlanner(RiskCalculator& risk_calculator)
    : risk_calculator_(risk_calculator) {
}

void FrenetPlanner::set_parameters(
    double max_speed,
    double max_accel,
    double max_curvature,
    double dt,
    double d_t_s,
    int n_s_sample,
    double target_speed,
    double safety_margin
) {
    max_speed_ = 10.0;
    max_accel_ = max_accel;
    max_curvature_ = max_curvature;
    dt_ = dt;
    d_t_s_ = d_t_s;
    n_s_sample_ = n_s_sample;
    target_speed_ = target_speed;
    safety_margin_ = safety_margin;
}

nav_msgs::msg::Path FrenetPlanner::plan_local_path(
    const nav_msgs::msg::Path::SharedPtr reference_path,
    const std::vector<Obstacle>& obstacles,
    const geometry_msgs::msg::Twist& current_twist
) {
    nav_msgs::msg::Path local_path;
    local_path.header = reference_path->header;

    std::vector<double> reference_path_s(reference_path->poses.size(), 0.0);
    for (size_t i = 1; i < reference_path->poses.size(); ++i) {
        double dx = reference_path->poses[i].pose.position.x - reference_path->poses[i-1].pose.position.x;
        double dy = reference_path->poses[i].pose.position.y - reference_path->poses[i-1].pose.position.y;
        reference_path_s[i] = reference_path_s[i-1] + std::hypot(dx, dy);
    }

    VehicleState current_state;
    get_frenet_state(reference_path, current_twist, current_state);
    std::vector<FrenetTrajectory> frenet_paths = generate_frenet_paths(current_state, reference_path);

    int valid_count = 0;
    for (auto& fp : frenet_paths) {
        if (check_collision(fp, obstacles)) {
            fp.valid = false;
            continue;
        }

        if (!check_path_validity(fp, reference_path_s)) {
            fp.valid = false;
            continue;
        }

        fp.cost = risk_calculator_.calculate_cost(fp, obstacles);
        valid_count++;
    }

    RCLCPP_INFO(rclcpp::get_logger("frenet_planner"),
        "Valid paths: %d / %zu", valid_count, frenet_paths.size());

    auto best_path = std::min_element(
        frenet_paths.begin(),
        frenet_paths.end(),
        [](const FrenetTrajectory& a, const FrenetTrajectory& b) {
            if (!a.valid) return false;
            if (!b.valid) return true;
            return a.cost < b.cost;
        }
    );

    if (best_path != frenet_paths.end() && best_path->valid) {
        RCLCPP_INFO(rclcpp::get_logger("frenet_planner"),
            "Best path found with %zu points, cost=%.2f",
            best_path->points.size(), best_path->cost);
        for (const auto& point : best_path->points) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = local_path.header;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = 0.0;
            local_path.poses.push_back(pose);
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("frenet_planner"),
            "No valid path found!");
    }

    return local_path;
}

std::vector<FrenetTrajectory> FrenetPlanner::generate_frenet_paths(
    const VehicleState& current_state,
    const nav_msgs::msg::Path::SharedPtr& reference_path) {
    std::vector<FrenetTrajectory> longitudinal_trajectories;
    std::vector<FrenetTrajectory> frenet_paths;

    if (!reference_path || reference_path->poses.size() < 2) {
        return frenet_paths;
    }

    std::vector<double> ref_x;
    std::vector<double> ref_y;
    ref_x.reserve(reference_path->poses.size());
    ref_y.reserve(reference_path->poses.size());
    for (const auto& pose : reference_path->poses) {
        ref_x.push_back(pose.pose.position.x);
        ref_y.push_back(pose.pose.position.y);
    }
    ReferenceCurve reference_curve(ref_x, ref_y);

    double max_s_available = reference_curve.max_s();
    double min_t = 0.8 * max_s_available / target_speed_;
    double max_t = max_s_available / target_speed_;

    double max_t_for_path = max_s_available / std::max(1e-3, target_speed_);
    double max_t_adjusted = std::min(max_t, max_t_for_path);
    double min_t_adjusted = std::min(min_t, max_t_adjusted);

    size_t n_time_samples = static_cast<size_t>((max_t_adjusted - min_t_adjusted) / dt_) + 1;
    size_t n_lateral_samples = static_cast<size_t>((max_road_width_left_ + max_road_width_right_) / d_road_width_) + 1;
    
    int n_speed_sample = std::max(0, n_s_sample_);
    double dv = std::abs(d_t_s_);
    size_t n_speed_samples = static_cast<size_t>(2 * n_speed_sample + 1);

    longitudinal_trajectories.reserve(n_time_samples * n_speed_samples);

    for (double T = min_t_adjusted; T <= max_t_adjusted; T += dt_) {
        for (int i = -n_speed_sample; i <= n_speed_sample; ++i) {
            double tv = target_speed_ + static_cast<double>(i) * dv;
            if (tv < 0.0) {
                tv = 0.0;
            }
            FrenetTrajectory lon_traj = generate_longitudinal_trajectory(current_state, tv, T);
            longitudinal_trajectories.push_back(std::move(lon_traj));
        }

    }

    combine_trajectories(
        frenet_paths,
        longitudinal_trajectories,
        current_state,
        reference_curve,
        n_time_samples,
        n_lateral_samples,
        n_speed_samples
    );

    return frenet_paths;
}

FrenetTrajectory FrenetPlanner::generate_longitudinal_trajectory(
    const VehicleState& current_state,
    double target_speed,
    double T) {
    FrenetTrajectory trajectory;
    trajectory.valid = true;

    size_t n_points = static_cast<size_t>(T / dt_) + 1;
    trajectory.t.reserve(n_points);
    trajectory.s.reserve(n_points);
    trajectory.s_dot.reserve(n_points);
    trajectory.s_ddot.reserve(n_points);
    trajectory.s_dddot.reserve(n_points);

    QuarticPolynomial lon_qp(
        current_state.s,
        current_state.s_dot,
        current_state.s_ddot,
        target_speed,
        0.0,
        T
    );

    for (double t = 0.0; t <= T; t += dt_) {
        trajectory.t.push_back(t);
        trajectory.s.push_back(lon_qp.calc_point(t));
        trajectory.s_dot.push_back(lon_qp.calc_first_derivative(t));
        trajectory.s_ddot.push_back(lon_qp.calc_second_derivative(t));
        trajectory.s_dddot.push_back(lon_qp.calc_third_derivative(t));
    }

    trajectory.cost = 0.0;

    return trajectory;
}

void FrenetPlanner::combine_trajectories(
    std::vector<FrenetTrajectory>& frenet_paths,
    const std::vector<FrenetTrajectory>& longitudinal_trajectories,
    const VehicleState& current_state,
    const ReferenceCurve& reference_curve,
    size_t n_time_samples,
    size_t n_lateral_samples,
    size_t n_speed_samples
) {
    frenet_paths.reserve(n_time_samples * n_lateral_samples * n_speed_samples);

    for (size_t i = 0; i < n_time_samples; ++i) {
        const size_t lon_base = i * n_speed_samples;

        for (size_t j = 0; j < n_lateral_samples; ++j) {
            const double target_d = -max_road_width_left_ + static_cast<double>(j) * d_road_width_;
            for (size_t k = 0; k < n_speed_samples; ++k) {
                const auto& lon = longitudinal_trajectories[lon_base + k];

                if (lon.t.empty()) {
                    continue;
                }

                FrenetTrajectory lat = generate_lateral_trajectory(current_state, target_d, lon);
                if (lat.t.size() != lon.t.size()) {
                    continue;
                }

                FrenetTrajectory fp;
                fp.t = lat.t;
                fp.d = lat.d;
                fp.d_s = lat.d_s;
                fp.d_ss = lat.d_ss;
                fp.d_dot = lat.d_dot;
                fp.d_ddot = lat.d_ddot;
                fp.d_dddot = lat.d_dddot;
                fp.s = lon.s;
                fp.s_dot = lon.s_dot;
                fp.s_ddot = lon.s_ddot;
                fp.s_dddot = lon.s_dddot;
                fp.cost = 0.0;
                fp.valid = true;
                fp.points.reserve(fp.s.size());

                for (size_t m = 0; m < fp.s.size(); ++m) {
                    FrenetState frenet_state{
                        fp.s[m],
                        fp.s_dot[m],
                        fp.s_ddot[m],
                        fp.d[m],
                        fp.d_s[m],
                        fp.d_ss[m]
                    };
                    CartesianState cartesian_state{};
                    frenet_to_cartesian(reference_curve, frenet_state, cartesian_state);

                    TrajectoryPoint point;
                    point.s = fp.s[m];
                    point.d = fp.d[m];
                    point.x = cartesian_state.x;
                    point.y = cartesian_state.y;
                    point.yaw = cartesian_state.yaw;
                    point.curvature = cartesian_state.curvature;
                    point.velocity = cartesian_state.velocity;
                    point.acceleration = cartesian_state.acceleration;

                    fp.points.push_back(point);
                }

                frenet_paths.push_back(std::move(fp));
            }
        }
    }
}

FrenetTrajectory FrenetPlanner::generate_lateral_trajectory(
    const VehicleState& current_state,
    double target_d,
    const FrenetTrajectory& longitudinal_traj) {
    FrenetTrajectory trajectory;
    trajectory.valid = true;

    if (longitudinal_traj.s.empty()) {
        trajectory.valid = false;
        return trajectory;
    }

    const double T = longitudinal_traj.t.back();
    QuinticPolynomial lat_qp(
        current_state.d,
        current_state.d_dot,
        current_state.d_ddot,
        target_d,
        0.0,
        0.0,
        T
    );

    const size_t n_points = longitudinal_traj.s.size();
    trajectory.t = longitudinal_traj.t;
    trajectory.s = longitudinal_traj.s;
    trajectory.s_dot = longitudinal_traj.s_dot;
    trajectory.s_ddot = longitudinal_traj.s_ddot;
    trajectory.s_dddot = longitudinal_traj.s_dddot;
    trajectory.d.reserve(n_points);
    trajectory.d_s.reserve(n_points);
    trajectory.d_ss.reserve(n_points);
    trajectory.d_dot.reserve(n_points);
    trajectory.d_ddot.reserve(n_points);
    trajectory.d_dddot.reserve(n_points);

    for (size_t i = 0; i < n_points; ++i) {
        const double t = longitudinal_traj.t[i];
        const double d = lat_qp.calc_point(t);
        const double d_dot_t = lat_qp.calc_first_derivative(t);
        const double d_ddot_t = lat_qp.calc_second_derivative(t);
        const double d_dddot_t = lat_qp.calc_third_derivative(t);

        const double s_dot = longitudinal_traj.s_dot[i];
        const double s_ddot = longitudinal_traj.s_ddot[i];
        const double s_dot_inv = 1.0 / (std::abs(s_dot) + 1e-6);
        const double s_dot_inv_sq = s_dot_inv * s_dot_inv;

        const double d_s_i = d_dot_t * s_dot_inv;
        const double d_ss_i = (d_ddot_t - d_s_i * s_ddot) * s_dot_inv_sq;

        trajectory.d.push_back(d);
        trajectory.d_s.push_back(d_s_i);
        trajectory.d_ss.push_back(d_ss_i);
        trajectory.d_dot.push_back(d_dot_t);
        trajectory.d_ddot.push_back(d_ddot_t);
        trajectory.d_dddot.push_back(d_dddot_t);
    }

    trajectory.cost = 0.0;
    return trajectory;
}

bool FrenetPlanner::check_collision(
    const FrenetTrajectory& trajectory,
    const std::vector<Obstacle>& obstacles
) {
    double collision_threshold = robot_radius_ + safety_margin_;

    for (const auto& point : trajectory.points) {
        for (const auto& obs : obstacles) {
            double dx = point.x - obs.x;
            double dy = point.y - obs.y;
            double distance = std::hypot(dx, dy);

            if (distance < collision_threshold) {
                return true;
            }
        }
    }

    return false;
}

bool FrenetPlanner::check_path_validity(
    const FrenetTrajectory& trajectory,
    const std::vector<double>& reference_path_s) {
    for (const auto& point : trajectory.points) {
        if (point.velocity > max_speed_) {
            return false;
        }

        if (std::abs(point.acceleration) > max_accel_) {
            return false;
        }

        if (std::abs(point.curvature) > max_curvature_) {
            return false;
        }
    }

    return true;
}

void FrenetPlanner::get_frenet_state(const nav_msgs::msg::Path::SharedPtr& path, const geometry_msgs::msg::Twist& current_twist, VehicleState& state) {
    state.s = 0.0;
    state.d = 0.0;
    state.d_ddot = 0.0;
    state.s_ddot = 0.0;

    if (!path || path->poses.size() < 2) {
        state.s_dot = 0.0;
        state.d_dot = 0.0;
        return;
    }

    const double path1_x = path->poses[1].pose.position.x - path->poses[0].pose.position.x;
    const double path1_y = path->poses[1].pose.position.y - path->poses[0].pose.position.y;
    const double path_norm = std::hypot(path1_x, path1_y);
    const double tx = (path_norm > 1e-6) ? (path1_x / path_norm) : 1.0;
    const double ty = (path_norm > 1e-6) ? (path1_y / path_norm) : 0.0;
    const double nx = -ty;
    const double ny = tx;

    const double vx = current_twist.linear.x;
    const double vy = current_twist.linear.y;

    state.s_dot = vx * tx + vy * ty;
    state.d_dot = vx * nx + vy * ny;

}

void FrenetPlanner::frenet_to_cartesian(
    const ReferenceCurve& reference_curve,
    const FrenetState& state,
    CartesianState& out
) {
    double rx, ry, rtheta, rkappa, rdkappa;
    reference_curve.sample(state.s, rx, ry, rtheta, rkappa, rdkappa);

    double cos_theta_r = std::cos(rtheta);
    double sin_theta_r = std::sin(rtheta);

    out.x = rx - sin_theta_r * state.d;
    out.y = ry + cos_theta_r * state.d;

    double one_minus_kappa_r_d = 1.0 - rkappa * state.d;
    double tan_delta_theta = state.d_s / one_minus_kappa_r_d;
    double delta_theta = std::atan2(state.d_s, one_minus_kappa_r_d);
    double cos_delta_theta = std::cos(delta_theta);

    out.yaw = delta_theta + rtheta;

    double kappa_r_d_prime = rdkappa * state.d + rkappa * state.d_s;

    out.curvature = (((state.d_ss + kappa_r_d_prime * tan_delta_theta) *
                      cos_delta_theta * cos_delta_theta) / one_minus_kappa_r_d + rkappa) *
                cos_delta_theta / one_minus_kappa_r_d;

    double d_dot = state.d_s * state.s_dot;
    out.velocity = std::sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d * state.s_dot * state.s_dot + d_dot * d_dot);

    double delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * out.curvature - rkappa;

    out.acceleration = (state.s_ddot * one_minus_kappa_r_d / cos_delta_theta +
                        state.s_dot * state.s_dot / cos_delta_theta *
                        (state.d_s * delta_theta_prime - kappa_r_d_prime));
}

}  // namespace frenet_planner
