#pragma once

#include "vision_lane_planner/visibility_control.h"

#include <builtin_interfaces/msg/time.hpp>
#include <camera_utility/camera_utility.hpp>

#include <nav_msgs/msg/path.hpp>

#include <tf2/LinearMath/Transform.h>

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace vision_lane_planner {

// 学習側の契約。learning/Models/model_components/vision_planner/vision_planner_network.py と
// learning/Models/data_utils/vision_planner/projection.py に一致させること
inline constexpr std::size_t num_slots     = 3;   // straight / left / right
inline constexpr std::size_t num_rows      = 48;  // input_height / 8
inline constexpr int         input_width   = 640;
inline constexpr int         input_height  = 384;
inline constexpr int         source_width  = 640;
inline constexpr int         source_height = 360;
inline constexpr int         pad_top       = (input_height - source_height) / 2;  // 12
inline constexpr int         pad_left      = (input_width - source_width) / 2;    // 0

// 学習側 row_anchor_targets は height // num_rows の整数 floor 除算。
// 割り切れない値になると、エラーなく行アンカー→画素の対応だけがずれる
static_assert(input_height % num_rows == 0, "input_height は num_rows で割り切れる必要があります");

// 行アンカーの地面距離は row25=14.9m の次が row24=95.2m と飛ぶ(実測)。
// 15m で打ち切ることで地平線直下の発散点を経路に混ぜない
inline constexpr double max_range_m = 15.0;
// 打ち切り後も、離れた2点を線形補間して存在しない直線を作らないための上限
inline constexpr double max_point_gap_m = 10.0;

enum class Slot : std::size_t {
    Straight = 0,
    Left     = 1,
    Right    = 2,
};

// nav_cmd で指示されたスロットの exist が閾値未満だったときに降格していく順序。
// 運用でいじる値ではないので max_range_m 等と同じく定数で持ち、パラメータ化しない
inline constexpr std::array<Slot, num_slots> fallback_order{Slot::Right, Slot::Left, Slot::Straight};

struct SlotPrediction {
    std::array<float, num_slots>                       exist{};        // ONNX 出力 exist (logit)
    std::array<std::array<float, num_rows>, num_slots> valid_logit{};  // ONNX 出力 valid (logit)
    std::array<std::array<float, num_rows>, num_slots> position{};     // ONNX 出力 position [0,1]
    bool                                               valid = false;  // 推論が成功したか
};

struct PathBuilderParams {
    double      exist_threshold;
    double      valid_threshold;
    double      path_resample_interval_m;
    std::size_t min_path_points;
};

// 経路が組めなかったことを下流へ伝えるための戻り値。
// valid=false のときも path のヘッダは埋まっており、poses だけが空になる
struct PathResult {
    nav_msgs::msg::Path path;
    bool                valid = false;
};

struct Point2D {
    double x;
    double y;
};

VISION_LANE_PLANNER_PUBLIC
Slot parse_slot(const std::string& command);

VISION_LANE_PLANNER_PUBLIC
std::optional<Slot> select_slot(
    const SlotPrediction& prediction,
    Slot                  commanded,
    double                exist_threshold);

// letterbox 後の画像における行アンカーの v 座標
VISION_LANE_PLANNER_PUBLIC
double row_anchor_v(std::size_t row);

// 選択スロットの有効行を地面へ投影し、近い順(x 昇順)に返す
VISION_LANE_PLANNER_PUBLIC
std::vector<Point2D> project_slot_rows(
    const SlotPrediction&                   prediction,
    Slot                                    slot,
    const camera_utility::CameraIntrinsics& intrinsics,
    const tf2::Transform&                   base_T_camera,
    double                                  valid_threshold);

// 単調性・ギャップ・距離で打ち切り、等間隔にリサンプルする
VISION_LANE_PLANNER_PUBLIC
std::vector<Point2D> truncate_and_resample(
    const std::vector<Point2D>& points,
    double                      resample_interval_m);

VISION_LANE_PLANNER_PUBLIC
PathResult build_path(
    const SlotPrediction&                   prediction,
    Slot                                    commanded,
    const camera_utility::CameraIntrinsics& intrinsics,
    const tf2::Transform&                   base_T_camera,
    const PathBuilderParams&                params,
    const builtin_interfaces::msg::Time&    stamp);

}  // namespace vision_lane_planner
