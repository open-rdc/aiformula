#include "vector_e2e_planner/branch_counter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace vector_e2e_planner {

namespace {

using Line = std::vector<Point2D>;

// valid の先頭からの連続区間だけを使う。足りなければ nullopt
std::optional<Line> prefix(
    const std::array<Point2D, num_path_points>& points,
    const std::array<bool, num_path_points>&    valid,
    const std::size_t                           min_points) {
    std::size_t stop = num_path_points;
    for (std::size_t i = 0; i < num_path_points; ++i) {
        if (!valid[i]) {
            stop = i;
            break;
        }
    }
    if (stop < min_points) {
        return std::nullopt;
    }
    return Line(points.begin(), points.begin() + static_cast<std::ptrdiff_t>(stop));
}

// x<0 の点を切り落とす。x=0 をまたぐ線分は x=0 の交点で切る。前方に点が無ければ nullopt
std::optional<Line> crop_behind(const Line& points) {
    const auto ahead = [](const Point2D& p) { return p.x >= 0.0; };
    if (std::all_of(points.begin(), points.end(), ahead)) {
        return points;
    }
    if (std::none_of(points.begin(), points.end(), ahead)) {
        return std::nullopt;
    }
    const std::size_t first = static_cast<std::size_t>(std::find_if(points.begin(), points.end(), ahead) - points.begin());
    if (first == 0) {
        Line kept;
        std::copy_if(points.begin(), points.end(), std::back_inserter(kept), ahead);
        return kept;
    }
    const Point2D& a = points[first - 1];
    const Point2D& b = points[first];
    const double   t = b.x != a.x ? (0.0 - a.x) / (b.x - a.x) : 0.0;
    Line           cropped{Point2D{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)}};
    cropped.insert(cropped.end(), points.begin() + static_cast<std::ptrdiff_t>(first), points.end());
    return cropped;
}

// 各点の折れ線への最短距離と、最終線分上の無制限な射影パラメータ
void nearest_on_polyline(
    const Line& points, const Line& line, std::vector<double>& dist, std::vector<double>& t_last) {
    dist.assign(points.size(), std::numeric_limits<double>::infinity());
    t_last.assign(points.size(), 0.0);
    for (std::size_t i = 0; i < points.size(); ++i) {
        for (std::size_t j = 0; j + 1 < line.size(); ++j) {
            const Point2D& a       = line[j];
            const double   sx      = line[j + 1].x - a.x;
            const double   sy      = line[j + 1].y - a.y;
            const double   length2 = std::max(sx * sx + sy * sy, 1e-12);
            const double   rx      = points[i].x - a.x;
            const double   ry      = points[i].y - a.y;
            const double   t_raw   = (rx * sx + ry * sy) / length2;
            const double   t       = std::min(std::max(t_raw, 0.0), 1.0);
            const double   dx      = points[i].x - (a.x + t * sx);
            const double   dy      = points[i].y - (a.y + t * sy);
            dist[i]                = std::min(dist[i], std::sqrt(dx * dx + dy * dy));
            if (j + 2 == line.size()) {
                t_last[i] = t_raw;
            }
        }
    }
}

}  // namespace

std::optional<double> divergence_distance(
    const std::array<Point2D, num_path_points>& straight_center,
    const std::array<Point2D, num_path_points>& other_center,
    const std::array<bool, num_path_points>&    valid_straight,
    const std::array<bool, num_path_points>&    valid_other,
    const double                                sep,
    const std::size_t                           min_points) {
    auto straight_prefix = prefix(straight_center, valid_straight, min_points);
    auto other_prefix    = prefix(other_center, valid_other, min_points);
    if (!straight_prefix || !other_prefix) {
        return std::nullopt;
    }
    auto straight_opt = crop_behind(*straight_prefix);
    auto other_opt    = crop_behind(*other_prefix);
    if (!straight_opt || !other_opt) {
        return std::nullopt;
    }
    Line straight = *straight_opt;
    Line other    = *other_opt;

    const Point2D origin{0.0, 0.0};
    if (straight[0].x > 0.0 || std::abs(straight[0].y) > 1e-9) {
        straight.insert(straight.begin(), origin);
    }
    if (straight.size() < 2U) {
        return std::nullopt;
    }

    std::vector<double> dist;
    std::vector<double> t_last;
    nearest_on_polyline(other, straight, dist, t_last);
    std::vector<bool> beyond(t_last.size());
    for (std::size_t i = 0; i < t_last.size(); ++i) {
        beyond[i] = t_last[i] > 1.0 + 1e-6;
    }
    if (dist[0] > sep) {
        return 0.0;
    }
    if (!(other[0].x <= 1e-9)) {
        other.insert(other.begin(), origin);
        dist.insert(dist.begin(), 0.0);
        beyond.insert(beyond.begin(), false);
    }
    std::vector<double> arc(other.size(), 0.0);
    for (std::size_t i = 1; i < other.size(); ++i) {
        const double dx = other[i].x - other[i - 1].x;
        const double dy = other[i].y - other[i - 1].y;
        arc[i]          = arc[i - 1] + std::sqrt(dx * dx + dy * dy);
    }
    for (std::size_t i = 1; i < other.size(); ++i) {
        if (beyond[i]) {
            return std::nullopt;
        }
        if (dist[i] > sep) {
            const double span = dist[i] - dist[i - 1];
            const double t    = span > 1e-12 ? (sep - dist[i - 1]) / span : 1.0;
            return arc[i - 1] + t * (arc[i] - arc[i - 1]);
        }
    }
    return std::nullopt;
}

const char* to_string(const BranchKind kind) {
    switch (kind) {
        case BranchKind::Left:
            return "left";
        case BranchKind::Right:
            return "right";
        case BranchKind::Both:
            break;
    }
    return "both";
}

const char* to_string(const BranchReason reason) {
    switch (reason) {
        case BranchReason::Pass:
            return "pass";
        case BranchReason::Vanish:
            return "vanish";
        case BranchReason::Jump:
            break;
    }
    return "jump";
}

BranchCounter::BranchCounter(const CounterParams& params)
    : params_(params), last_fire_(-std::numeric_limits<double>::infinity()) {
}

BranchCounter::Found BranchCounter::observe(
    const std::array<double, num_slots>& exist_prob,
    const SlotLines&                     corridor_left,
    const SlotLines&                     corridor_right,
    const SlotFlags&                     valid_prob) const {
    std::array<std::array<Point2D, num_path_points>, num_slots> center{};
    std::array<std::array<bool, num_path_points>, num_slots>    valid{};
    for (std::size_t s = 0; s < num_slots; ++s) {
        for (std::size_t i = 0; i < num_path_points; ++i) {
            center[s][i] = Point2D{
                (corridor_left[s][i].x + corridor_right[s][i].x) / 2.0,
                (corridor_left[s][i].y + corridor_right[s][i].y) / 2.0};
            valid[s][i] = valid_prob[s][i] >= params_.valid_thr;
        }
    }
    Found found;
    for (std::size_t side = 0; side < 2; ++side) {
        const std::size_t slot = side + 1;  // 1: left, 2: right
        if (exist_prob[slot] < params_.exist_thr) {
            continue;
        }
        found[side] = divergence_distance(center[0], center[slot], valid[0], valid[slot], params_.sep, params_.min_points);
    }
    return found;
}

std::optional<BranchEvent> BranchCounter::update(
    const std::array<double, num_slots>& exist_prob,
    const SlotLines&                     corridor_left,
    const SlotLines&                     corridor_right,
    const SlotFlags&                     valid_prob,
    const double                         travel_m) {
    odometer_ += std::max(travel_m, 0.0);
    const Found found = observe(exist_prob, corridor_left, corridor_right, valid_prob);
    if (!track_) {
        update_candidate(found, travel_m);
        return std::nullopt;
    }
    return update_track(found, travel_m);
}

void BranchCounter::update_candidate(const Found& found, const double travel_m) {
    const CounterParams& p = params_;
    if (odometer_ - last_fire_ < p.min_gap_m) {
        candidate_.reset();
        return;
    }
    Found usable;
    for (std::size_t s = 0; s < 2; ++s) {
        if (found[s] && p.start_min_m <= *found[s] && *found[s] <= p.arm_dist_m) {
            usable[s] = found[s];
        }
    }
    if (!usable[0] && !usable[1]) {
        if (candidate_) {
            candidate_->miss += 1;
            candidate_->d -= travel_m;
            if (candidate_->miss > p.cand_miss_frames) {
                candidate_.reset();
            }
        }
        return;
    }
    std::size_t side = 0;
    if (!usable[0] || (usable[1] && *usable[1] < *usable[0])) {
        side = 1;
    }
    const double d = *usable[side];

    std::optional<Candidate> cand = candidate_;
    if (cand) {
        const double predicted = cand->d - travel_m;
        if (std::abs(d - predicted) <= p.gate_m + p.gate_ratio * std::max(predicted, 0.0)) {
            cand->n += 1;
            cand->d    = d;
            cand->miss = 0;
            for (std::size_t s = 0; s < 2; ++s) {
                if (usable[s] && *usable[s] <= d + p.side_margin_m) {
                    cand->sides[s] += 1;
                }
            }
        } else {
            cand.reset();
        }
    }
    if (!cand) {
        cand    = Candidate{};
        cand->d = d;
        for (std::size_t s = 0; s < 2; ++s) {
            if (usable[s] && *usable[s] <= d + p.side_margin_m) {
                cand->sides[s] += 1;
            }
        }
    }
    candidate_ = cand;
    if (cand->n >= p.confirm_frames) {
        Track track;
        track.d_pred = cand->d;
        track.n_obs  = cand->n;
        track.sides  = cand->sides;
        track_       = track;
        candidate_.reset();
    }
}

std::optional<BranchEvent> BranchCounter::update_track(const Found& found, const double travel_m) {
    const CounterParams& p = params_;
    Track&               t = *track_;
    t.d_pred -= travel_m;
    if (!found[0] && !found[1]) {
        t.miss += 1;
        t.miss_m += travel_m;
        t.near_obs = 0;
        if (t.d_pred < p.pass_dist_m) {
            return fire(BranchReason::Pass);
        }
        if (t.d_pred < p.near_m && t.miss >= p.vanish_frames) {
            return fire(BranchReason::Vanish);
        }
        if (t.miss_m >= p.lost_m) {
            track_.reset();
        }
        return std::nullopt;
    }
    t.miss   = 0;
    t.miss_m = 0.0;
    std::size_t side = 0;
    if (!found[0] || (found[1] && std::abs(*found[1] - t.d_pred) < std::abs(*found[0] - t.d_pred))) {
        side = 1;
    }
    const double d    = *found[side];
    const double gate = p.gate_m + p.gate_ratio * std::max(t.d_pred, 0.0);
    if (std::abs(d - t.d_pred) <= gate) {
        t.incons = 0;
        t.d_pred += p.gain * (d - t.d_pred);
        t.n_obs += 1;
        double dmin = std::numeric_limits<double>::infinity();
        for (const auto& value : found) {
            if (value) {
                dmin = std::min(dmin, *value);
            }
        }
        for (std::size_t s = 0; s < 2; ++s) {
            if (found[s] && *found[s] <= dmin + p.side_margin_m) {
                t.sides[s] += 1;
            }
        }
        t.near_obs = d <= p.pass_obs_m ? t.near_obs + 1 : 0;
        if (t.near_obs >= p.pass_obs_frames || t.d_pred < p.pass_dist_m) {
            return fire(BranchReason::Pass);
        }
        return std::nullopt;
    }
    if (t.d_pred < p.near_m && d > t.d_pred + p.jump_m) {
        return fire(BranchReason::Jump);
    }
    t.incons += 1;
    if (t.incons >= p.reinit_frames) {
        t.d_pred = d;
        t.incons = 0;
    } else if (t.d_pred < p.pass_dist_m) {
        return fire(BranchReason::Pass);
    }
    return std::nullopt;
}

BranchKind BranchCounter::track_kind(const Track& track) const {
    const int need = std::max(2, static_cast<int>(std::ceil(params_.side_frac * track.n_obs)));
    const bool left  = track.sides[0] >= need;
    const bool right = track.sides[1] >= need;
    if (left && right) {
        return BranchKind::Both;
    }
    if (left) {
        return BranchKind::Left;
    }
    if (right) {
        return BranchKind::Right;
    }
    return track.sides[1] > track.sides[0] ? BranchKind::Right : BranchKind::Left;
}

std::optional<BranchEvent> BranchCounter::fire(const BranchReason reason) {
    const Track track = *track_;
    track_.reset();
    candidate_.reset();
    if (odometer_ - last_fire_ < params_.min_gap_m) {
        return std::nullopt;
    }
    const BranchEvent event{track_kind(track), odometer_, count_, reason};
    count_ += 1;
    last_fire_ = odometer_;
    return event;
}

RoutePlan::RoutePlan(const std::vector<std::string>& commands) {
    if (commands.empty()) {
        throw std::invalid_argument("route_plan が空です");
    }
    for (const std::string& name : commands) {
        const auto slot = parse_slot(name);
        if (!slot) {
            throw std::invalid_argument("route_plan に未知のスロットがあります: " + name);
        }
        commands_.push_back(*slot);
    }
}

Slot RoutePlan::next_slot(const int k) const {
    return commands_[static_cast<std::size_t>(k) % commands_.size()];
}

}  // namespace vector_e2e_planner
