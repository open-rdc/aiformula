#!/usr/bin/env python3
"""カーブのきつさでサンプルに損失の重みを付ける。

MSE 回帰は迷う入力に対して候補の平均を出すのが最適なので、数の少ない急カーブほど
直進側に縮む。実機データ (real20260927_v2) では 10 m 先で横 6 m を超える正解が
99 件 (1%) あるのに、予測は最大 5.66 m で一度も 6 m を超えなかった (2026-09-28)。
数の少ないカーブの損失を重くして、平均に引かれる力を打ち消す。

mode:
  surprise : 曲がり量のヒストグラムから自己情報量 -log p を重みにする
             (feat/shannon_surprise と同じ方式)。重みはデータの分布で決まる
  none     : 全サンプル 1
5 つの瓶に固定の重みを付ける方式 (bins) も試したが、surprise のほうが急カーブに効き
直線の悪化も半分だったので消した (2026-09-28)。
train 全体での平均が 1 になるよう正規化する。重みの平均が変わると
実効的な学習率が変わってしまい、重み付けの効果と区別できなくなるため。
"""

from typing import Dict, List

import numpy as np


def curve_score(waypoints_m: np.ndarray) -> float:
    """経路の中で横ずれ |y| が最大になる点の y [m] (左 +)。

    waypoint は距離で切ってある (0.5 m x 20 点) ので、速度によらず
    同じ曲がり方なら同じ値になる
    """
    y = np.asarray(waypoints_m, dtype=np.float32)[:, 1]
    return float(y[int(np.argmax(np.abs(y)))])


class CurveWeighter:
    def __init__(self, config: Dict, train_scores: np.ndarray):
        self.mode = config.get('mode', 'none')
        scores = np.asarray(train_scores, dtype=np.float64)

        if self.mode == 'surprise':
            num_bins = max(2, int(config.get('num_bins', 16)))
            smoothing = max(0.0, float(config.get('smoothing', 1.0)))
            hist, bin_edges = np.histogram(scores, bins=num_bins)
            probs = (hist + smoothing) / (hist.sum() + smoothing * num_bins)
            self.edges = bin_edges[1:-1]
            table = np.minimum(-np.log(probs), float(config.get('max_weight', 50.0)))
            self.entropy_nats = float(-np.sum(probs * np.log(probs)))
        elif self.mode == 'none':
            self.edges = np.array([])
            table = np.ones(1)
        else:
            raise ValueError(f"curve_weighting.mode は 'none' / 'surprise': {self.mode}")

        # 瓶の重みは train 全体で平均 1 に正規化した値で持つ
        counts = np.bincount(self._bin(scores), minlength=len(table))
        mean = float((counts * table).sum() / max(1, counts.sum()))
        self.table = (table / mean).astype(np.float32)
        self.counts = counts

    def _bin(self, scores) -> np.ndarray:
        return np.digitize(np.asarray(scores), self.edges).astype(np.int64)

    def __call__(self, score: float) -> float:
        return float(self.table[int(self._bin([score])[0])])

    def describe(self) -> List[str]:
        if self.mode == 'none':
            return ['Curve weighting: none']
        lines = [f'Curve weighting: {self.mode} (train 平均 1 に正規化)']
        bounds = [-np.inf, *self.edges, np.inf]
        for k, (w, n) in enumerate(zip(self.table, self.counts)):
            lines.append(f'  bin {k:2d} [{bounds[k]:6.2f}, {bounds[k + 1]:6.2f}) m: '
                         f'n={int(n):5d}  weight={w:.3f}')
        if self.mode == 'surprise':
            lines.append(f'  entropy={self.entropy_nats:.3f} nats')
        return lines
