#pragma once

#include <cmath>

#include <Eigen/Dense>

namespace mpcc_controller {

static constexpr int NX  = 7;    // 状態次元 (X, Y, phi, v/vx, omega/delta, s, vs)
static constexpr int NU  = 3;    // 入力次元 (du0, du1, dvs)
static constexpr int NPC = 1;    // 多面体制約数 (トラック境界のみ)
static constexpr int NS  = 1;    // ソフト制約スラック数
static constexpr int N   = 50;   // ホライズン長 (変更時は再ビルド)
static constexpr int NB  = NX;   // ボックス制約の最大数

static constexpr double INF = 1e5;
static constexpr double PI  = M_PI;

// 状態・入力のインデックス定数 (モデル共通レイアウト)
struct StateInputIndex {
  // 状態
  int X    = 0;
  int Y    = 1;
  int phi  = 2;
  int v    = 3;   // DiffDrive: v,  Ackermann: vx
  int ctrl = 4;   // DiffDrive: omega_cmd,  Ackermann: delta
  int s    = 5;
  int vs   = 6;
  // 入力
  int du0  = 0;   // DiffDrive: dv,  Ackermann: a
  int du1  = 1;   // DiffDrive: domega,  Ackermann: ddelta
  int dvs  = 2;
  // 制約
  int con_track = 0;
};

static const StateInputIndex si_index;

}  // namespace mpcc_controller
