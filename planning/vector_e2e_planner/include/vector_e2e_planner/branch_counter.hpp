#pragma once

#include "vector_e2e_planner/path_builder.hpp"
#include "vector_e2e_planner/visibility_control.h"

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace vector_e2e_planner {

// モデル出力（走行帯）の形だけから「何番目の分岐点を通過したか」を数える。
// 直進と他スロットの走行帯中心線が分かれる地点までの距離を観測し、走行距離で予測しながら追跡して、
// 通過（距離が 0 になる・近距離で観測が消える・遠くへ跳ぶ）を検出する

// 直進と他スロットの走行帯中心線が分かれる地点の、other に沿った弧長 [m]。
// 後方（x<0）は切り落とし、先頭がすでに sep 超なら 0（通過済み）。判定できなければ nullopt
VECTOR_E2E_PLANNER_PUBLIC
std::optional<double> divergence_distance(
    const std::array<Point2D, num_path_points>& straight_center,
    const std::array<Point2D, num_path_points>& other_center,
    const std::array<bool, num_path_points>&    valid_straight,
    const std::array<bool, num_path_points>&    valid_other,
    double                                      sep,
    std::size_t                                 min_points);

struct CounterParams {
    double      exist_thr       = 0.5;   // slot の exist 確率がこれ以上のものだけ使う
    double      valid_thr       = 0.5;   // path_valid 確率の閾値
    double      sep             = 1.0;   // 分岐とみなす走行帯中心線どうしの横離れ [m]
    std::size_t min_points      = 3;     // 有効点数の下限
    double      start_min_m     = 2.0;   // 追跡開始に必要な最小の観測距離
    double      arm_dist_m      = 18.0;  // 追跡を始める最大の観測距離
    int         confirm_frames  = 3;     // 追跡開始までに必要な連続の整合観測フレーム数
    int         cand_miss_frames = 2;    // 候補中に許す観測欠落フレーム数
    double      gate_m          = 2.5;   // 観測と予測の整合とみなす差
    double      gate_ratio      = 0.2;
    double      gain            = 0.5;   // 観測で予測距離を更新するときの混合率
    int         reinit_frames   = 3;     // 不整合が続いたら観測へ張り替えるフレーム数
    double      pass_dist_m     = 0.0;   // 予測距離がこれ未満になったら通過
    double      pass_obs_m      = 0.25;  // 整合した観測距離がこれ以下の状態が続いたら通過
    int         pass_obs_frames = 2;
    double      near_m          = 6.0;   // 「近距離」の定義
    int         vanish_frames   = 2;     // 近距離で観測が消えたら通過とみなすフレーム数
    double      jump_m          = 6.0;   // 近距離から観測がこれ以上遠くへ跳んだら通過
    double      lost_m          = 12.0;  // 観測が無いまま走行距離がこれを超えたら追跡を破棄
    double      min_gap_m       = 5.0;   // イベント間の最小走行距離 [m]
    double      side_margin_m   = 2.0;   // 左右が同じ分岐点とみなす観測距離の差
    double      side_frac       = 0.3;   // 種類に含める側の最小出現率
};

enum class BranchKind { Left, Right, Both };
enum class BranchReason { Pass, Vanish, Jump };

VECTOR_E2E_PLANNER_PUBLIC
const char* to_string(BranchKind kind);

VECTOR_E2E_PLANNER_PUBLIC
const char* to_string(BranchReason reason);

struct BranchEvent {
    BranchKind   kind;
    double       travel_m;  // 発火時のオドメータ積算 [m]
    int          index;     // 0 始まりのイベント番号
    BranchReason reason;
};

// フレームごとに update() を呼ぶ状態機械。分岐点を通過したフレームでだけ BranchEvent を返す
class VECTOR_E2E_PLANNER_PUBLIC BranchCounter {
   public:
    explicit BranchCounter(const CounterParams& params = CounterParams());

    std::optional<BranchEvent> update(
        const std::array<double, num_slots>& exist_prob,
        const SlotLines&                     corridor_left,
        const SlotLines&                     corridor_right,
        const SlotFlags&                     valid_prob,
        double                               travel_m);

    int count() const { return count_; }

   private:
    // 左右それぞれの分岐までの観測距離。観測できなかった側は nullopt
    using Found = std::array<std::optional<double>, 2>;

    struct Candidate {
        int                n     = 1;
        double             d     = 0.0;
        int                miss  = 0;
        std::array<int, 2> sides = {0, 0};
    };

    struct Track {
        double             d_pred = 0.0;
        int                n_obs  = 0;
        int                miss   = 0;
        double             miss_m = 0.0;
        int                incons = 0;
        int                near_obs = 0;
        std::array<int, 2> sides  = {0, 0};
    };

    Found                      observe(
                            const std::array<double, num_slots>& exist_prob,
                            const SlotLines& corridor_left, const SlotLines& corridor_right,
                            const SlotFlags& valid_prob) const;
    void                       update_candidate(const Found& found, double travel_m);
    std::optional<BranchEvent> update_track(const Found& found, double travel_m);
    std::optional<BranchEvent> fire(BranchReason reason);
    BranchKind                 track_kind(const Track& track) const;

    CounterParams              params_;
    int                        count_    = 0;
    double                     odometer_ = 0.0;
    std::optional<Track>       track_;
    double                     last_fire_;
    std::optional<Candidate>   candidate_;
};

// 交差点インデックス k（通過済みイベント数）→ 次の分岐点で選ぶ slot。指示列は lap ごとに繰り返す
class VECTOR_E2E_PLANNER_PUBLIC RoutePlan {
   public:
    // 空・未知のスロット名は std::invalid_argument
    explicit RoutePlan(const std::vector<std::string>& commands);

    Slot next_slot(int k) const;

   private:
    std::vector<Slot> commands_;
};

}  // namespace vector_e2e_planner
