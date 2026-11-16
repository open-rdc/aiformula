#pragma once

#include "vector_e2e_planner/visibility_control.h"

#include <builtin_interfaces/msg/time.hpp>

#include <nav_msgs/msg/path.hpp>

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace vector_e2e_planner {

// 学習側の契約。learning/Models/vector_planner/model/heads.py と deploy/export.py に一致させること
inline constexpr std::size_t num_slots        = 3;   // straight / left / right
inline constexpr std::size_t num_path_points  = 40;  // 経路・走行帯の点数（0.5 m 刻み）
inline constexpr std::size_t num_signal_class = 4;   // none / red / green / off
inline constexpr std::size_t signal_red_index = 1;
inline constexpr int         input_width      = 640;
inline constexpr int         input_height     = 384;
inline constexpr int         source_width     = 640;
inline constexpr int         source_height    = 360;
inline constexpr int         pad_top          = (input_height - source_height) / 2;  // 12
inline constexpr int         pad_left         = (input_width - source_width) / 2;    // 0

enum class Slot : std::size_t {
    Straight = 0,
    Left     = 1,
    Right    = 2,
};

struct Point2D {
    double x;
    double y;
};

struct Pose2D {
    double x;
    double y;
    double yaw;
};

using SlotLines = std::array<std::array<Point2D, num_path_points>, num_slots>;
using SlotFlags = std::array<std::array<double, num_path_points>, num_slots>;

// ONNX 出力のうち経路選択・信号停止に使うものだけ。座標は base_link（x 前方・y 左）[m]
struct ModelOutputs {
    std::array<double, num_slots>        branch_exist{};  // sigmoid 済み
    SlotLines                            path_xy{};
    SlotFlags                            path_valid{};    // sigmoid 済み
    SlotLines                            corridor_left{};
    SlotLines                            corridor_right{};
    std::array<double, num_signal_class> signal_prob{};   // none / red / green / off
    Point2D                              signal_xy{0.0, 0.0};  // パネル直下の地面点
    bool                                 valid      = false;   // 推論が成功したか
};

VECTOR_E2E_PLANNER_PUBLIC
std::optional<Slot> parse_slot(const std::string& name);

VECTOR_E2E_PLANNER_PUBLIC
const char* slot_name(Slot slot);

// 指示スロットの branch_exist が閾値未満なら存在しない分岐を選ばず straight にする
VECTOR_E2E_PLANNER_PUBLIC
Slot select_slot(Slot commanded, const std::array<double, num_slots>& branch_exist, double exist_threshold);

// path_valid が閾値以上の先頭からの連続区間だけを返す（非有限値もそこで打ち切る）
VECTOR_E2E_PLANNER_PUBLIC
std::vector<Point2D> valid_prefix(
    const std::array<Point2D, num_path_points>& path,
    const std::array<double, num_path_points>&  valid_prob,
    double                                      threshold);

// モデル経路の時間 EMA（自車運動補償つき）。前回出力は odom 座標で保持する。
// 式は alpha*new + (1-alpha)*prev。スロット切替・非有限値・初回は新しい経路をそのまま採る
class VECTOR_E2E_PLANNER_PUBLIC PathSmoother {
   public:
    explicit PathSmoother(double alpha);

    std::array<Point2D, num_path_points> step(
        const std::array<Point2D, num_path_points>& path, Slot slot, const Pose2D& odom_T_base);

   private:
    double                                              alpha_;
    std::optional<std::array<Point2D, num_path_points>> previous_world_;
    std::optional<Slot>                                 previous_slot_;
};

// 原点（base_link）を先頭に足し、Catmull-Rom で interval 程度の間隔へ補間する。interval <= 0 なら補間しない
VECTOR_E2E_PLANNER_PUBLIC
std::vector<Point2D> build_output_points(const std::vector<Point2D>& points, double interval);

// target を経路へ射影した足までの、経路先頭からの弧長 − offset（下限 0）。経路末端より先は最終線分の延長で測る
VECTOR_E2E_PLANNER_PUBLIC
std::optional<double> stop_distance(const std::vector<Point2D>& points, const Point2D& target, double offset);

VECTOR_E2E_PLANNER_PUBLIC
std::vector<Point2D> to_world(const std::vector<Point2D>& points, const Pose2D& pose);

// points は frame_id の座標。向きは次点への方位（最終点は直前の向き）
VECTOR_E2E_PLANNER_PUBLIC
nav_msgs::msg::Path make_path(
    const std::vector<Point2D>& points, const std::string& frame_id, const builtin_interfaces::msg::Time& stamp);

}  // namespace vector_e2e_planner
