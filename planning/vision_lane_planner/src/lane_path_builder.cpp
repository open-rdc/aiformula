#include "vision_lane_planner/lane_path_builder.hpp"

#include <opencv2/core.hpp>
#include <utilities/utils.hpp>

#include <cmath>

namespace vision_lane_planner {

namespace {

double sigmoid(const float logit) {
    return 1.0 / (1.0 + std::exp(-static_cast<double>(logit)));
}

Point2D interpolate_at(
    const std::vector<Point2D>& points,
    const std::vector<double>&  arc_length,
    const double                target) {
    for (std::size_t i = 1; i < points.size(); ++i) {
        if (target > arc_length[i]) {
            continue;
        }
        const double span  = arc_length[i] - arc_length[i - 1];
        const double ratio = span < 1.0e-9 ? 0.0 : (target - arc_length[i - 1]) / span;
        return Point2D{
            points[i - 1].x + ratio * (points[i].x - points[i - 1].x),
            points[i - 1].y + ratio * (points[i].y - points[i - 1].y)};
    }
    return points.back();
}

}  // namespace

Slot parse_slot(const std::string& command) {
    if (command == "left") {
        return Slot::Left;
    }
    if (command == "right") {
        return Slot::Right;
    }
    return Slot::Straight;
}

std::optional<Slot> select_slot(
    const SlotPrediction& prediction,
    const Slot            commanded,
    const double          exist_threshold) {
    const auto exists = [&prediction, exist_threshold](const Slot slot) {
        return sigmoid(prediction.exist[static_cast<std::size_t>(slot)]) > exist_threshold;
    };

    if (exists(commanded)) {
        return commanded;
    }
    for (const Slot slot : fallback_order) {
        if (slot == commanded) {
            continue;
        }
        if (exists(slot)) {
            return slot;
        }
    }
    return std::nullopt;
}

double row_anchor_v(const std::size_t row) {
    constexpr double row_height = static_cast<double>(input_height) / static_cast<double>(num_rows);
    return static_cast<double>(row) * row_height + (row_height - 1.0) / 2.0;
}

std::vector<Point2D> project_slot_rows(
    const SlotPrediction&                   prediction,
    const Slot                              slot,
    const camera_utility::CameraIntrinsics& intrinsics,
    const tf2::Transform&                   base_T_camera,
    const double                            valid_threshold) {
    const std::size_t slot_index = static_cast<std::size_t>(slot);

    std::vector<Point2D> points;
    points.reserve(num_rows);
    for (std::size_t offset = 0; offset < num_rows; ++offset) {
        const std::size_t row = num_rows - 1 - offset;
        if (sigmoid(prediction.valid_logit[slot_index][row]) <= valid_threshold) {
            continue;
        }

        // 行アンカーは letterbox 後の座標なので、pad を引いて元画像の座標へ戻す
        const double u = static_cast<double>(prediction.position[slot_index][row]) *
                             static_cast<double>(input_width - 1) -
                         static_cast<double>(pad_left);
        const double v = row_anchor_v(row) - static_cast<double>(pad_top);
        // 最終画素行より外へ外挿しないので上限は height - 1。
        // row 46 (v=359.5) と row 47 (v=367.5) はここで落ちる
        if (u < 0.0 || u > static_cast<double>(intrinsics.width - 1) || v < 0.0 ||
            v > static_cast<double>(intrinsics.height - 1)) {
            continue;
        }

        tf2::Vector3 base_point;
        if (!camera_utility::pixelToPoint(
                cv::Point2f(static_cast<float>(u), static_cast<float>(v)), intrinsics, base_T_camera, base_point,
                0.0)) {
            continue;
        }
        points.push_back(Point2D{base_point.x(), base_point.y()});
    }
    return points;
}

std::vector<Point2D> truncate_and_resample(
    const std::vector<Point2D>& points,
    const double                resample_interval_m) {
    std::vector<Point2D> kept;
    kept.reserve(points.size());
    for (const Point2D& point : points) {
        if (point.x > max_range_m) {
            break;
        }
        if (kept.empty()) {
            // まだ1点も採用していない間だけ読み飛ばす(自車の真下・後方は使わない)。
            // 採用済みなら下の単調性チェックが先に break するのでここには来ない
            if (point.x <= 0.0) {
                continue;
            }
        } else {
            if (point.x <= kept.back().x) {
                break;  // 地平線付近の発散で経路が折り返すのを防ぐ
            }
            if (std::hypot(point.x - kept.back().x, point.y - kept.back().y) > max_point_gap_m) {
                break;  // 離れた2点を補間して存在しない直線を作らない
            }
        }
        kept.push_back(point);
    }

    if (kept.size() < 2U) {
        return kept;
    }

    std::vector<double> arc_length(kept.size(), 0.0);
    for (std::size_t i = 1; i < kept.size(); ++i) {
        arc_length[i] = arc_length[i - 1] +
                        std::hypot(kept[i].x - kept[i - 1].x, kept[i].y - kept[i - 1].y);
    }

    std::vector<Point2D> resampled;
    resampled.reserve(
        static_cast<std::size_t>(std::ceil(arc_length.back() / resample_interval_m)) + 2U);
    for (double s = 0.0; s < arc_length.back(); s += resample_interval_m) {
        resampled.push_back(interpolate_at(kept, arc_length, s));
    }
    resampled.push_back(kept.back());
    return resampled;
}

PathResult build_path(
    const SlotPrediction&                   prediction,
    const Slot                              commanded,
    const camera_utility::CameraIntrinsics& intrinsics,
    const tf2::Transform&                   base_T_camera,
    const PathBuilderParams&                params,
    const builtin_interfaces::msg::Time&    stamp) {
    // 経路が組めなくてもヘッダは必ず埋める。publish を止めると下流が古い経路を
    // ラッチしたまま走り続け、「見えていない」ことと正常が区別できなくなる
    PathResult result;
    result.path.header.stamp    = stamp;
    result.path.header.frame_id = "base_link";

    // 推論が失敗した SlotPrediction は exist が全て 0 で、sigmoid(0)=0.5 になる。
    // exist_threshold が 0.5 未満だと select_slot が straight を「正常な選択」として返してしまう
    if (!prediction.valid) {
        return result;
    }

    const auto slot = select_slot(prediction, commanded, params.exist_threshold);
    if (!slot) {
        return result;
    }

    const auto projected =
        project_slot_rows(prediction, *slot, intrinsics, base_T_camera, params.valid_threshold);
    const auto samples = truncate_and_resample(projected, params.path_resample_interval_m);
    if (samples.size() < params.min_path_points || samples.size() < 2U) {
        return result;
    }

    result.path.poses.reserve(samples.size());
    double yaw = 0.0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        if (i + 1 < samples.size()) {
            yaw = std::atan2(samples[i + 1].y - samples[i].y, samples[i + 1].x - samples[i].x);
        }
        geometry_msgs::msg::PoseStamped pose;
        pose.header           = result.path.header;
        pose.pose.position.x  = samples[i].x;
        pose.pose.position.y  = samples[i].y;
        pose.pose.position.z  = 0.0;
        pose.pose.orientation = utils::yaw_to_quaternion(yaw);
        result.path.poses.push_back(pose);
    }
    result.valid = true;
    return result;
}

}  // namespace vision_lane_planner
