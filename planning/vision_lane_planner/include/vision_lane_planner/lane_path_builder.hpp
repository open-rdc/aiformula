#pragma once

#include "vision_lane_planner/visibility_control.h"

#include <builtin_interfaces/msg/time.hpp>
#include <camera_utility/camera_utility.hpp>

#include <nav_msgs/msg/path.hpp>

#include <tf2/LinearMath/Transform.h>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace vision_lane_planner {

// 学習側の契約。learning/Models/model_components/vision_planner/vision_planner_head.py と
// learning/Models/data_utils/vision_planner/projection.py に一致させること
inline constexpr std::size_t num_slots        = 3;   // straight / left / right
inline constexpr std::size_t row_anchor_count = 48;  // 行アンカーの刻みを決める値。input_height / 8
inline constexpr std::size_t row_start        = 25;  // ONNX 出力行 0 が対応する絶対行(vision_planner_head.py の ROW_START)
inline constexpr std::size_t num_rows         = row_anchor_count - row_start;  // ONNX 出力の行数
inline constexpr int         input_width   = 640;
inline constexpr int         input_height  = 384;
inline constexpr int         source_width  = 640;
inline constexpr int         source_height = 360;
inline constexpr int         pad_top       = (input_height - source_height) / 2;  // 12
inline constexpr int         pad_left      = (input_width - source_width) / 2;    // 0

// 学習側 row_anchor_targets は height // row_anchor_count の整数 floor 除算。
// 割り切れない値になると、エラーなく行アンカー→画素の対応だけがずれる
static_assert(input_height % row_anchor_count == 0, "input_height は row_anchor_count で割り切れる必要があります");

// 行アンカーの地面距離は絶対行25(出力行0)=14.9m の次の絶対行24(教師なし)=95.2m へ飛ぶ(実測)。
// 15m で打ち切ることで地平線直下の発散点を経路に混ぜない
inline constexpr double max_range_m = 15.0;
// 打ち切り後も、離れた2点を線形補間して存在しない直線を作らないための上限
inline constexpr double max_point_gap_m = 10.0;

// sigmoid(valid_logit) がこの値を超えた行だけ有効とみなす
inline constexpr double valid_threshold = 0.5;

enum class Slot : std::size_t {
    Straight = 0,
    Left     = 1,
    Right    = 2,
};

struct SlotPrediction {
    std::array<std::array<float, num_rows>, num_slots> position{};     // ONNX 出力 position [0,1]
    std::array<std::array<float, num_rows>, num_slots> valid_logit{};  // ONNX 出力 valid（ロジット）
    bool                                               valid = false;  // 推論が成功したか
};

// 経路が組めなかったことを下流へ伝えるための戻り値。
// 組めなかったときも path のヘッダは埋まっており、poses だけが空になる
struct PathResult {
    nav_msgs::msg::Path path;
};

struct Point2D {
    double x;
    double y;
};

VISION_LANE_PLANNER_PUBLIC
Slot parse_slot(const std::string& command);

// letterbox 後の画像における行アンカーの v 座標。row は出力行(0..num_rows-1)で、絶対行は row_start + row
VISION_LANE_PLANNER_PUBLIC
double row_anchor_v(std::size_t row);

// 選択スロットの行を地面へ投影し、近い順(x 昇順)に返す
VISION_LANE_PLANNER_PUBLIC
std::vector<Point2D> project_slot_rows(
    const SlotPrediction&                   prediction,
    Slot                                    slot,
    const camera_utility::CameraIntrinsics& intrinsics,
    const tf2::Transform&                   base_T_camera);

// 単調性・ギャップ・距離で打ち切る
VISION_LANE_PLANNER_PUBLIC
std::vector<Point2D> truncate(const std::vector<Point2D>& points);

VISION_LANE_PLANNER_PUBLIC
PathResult build_path(
    const SlotPrediction&                   prediction,
    Slot                                    commanded,
    const camera_utility::CameraIntrinsics& intrinsics,
    const tf2::Transform&                   base_T_camera,
    const builtin_interfaces::msg::Time&    stamp);

}  // namespace vision_lane_planner
