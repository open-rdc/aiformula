"""右左折の手前とコーンを見つけたときに出す速度上限。

speed_path_planner は経路の曲率だけで速度を決めるので、分岐の右左折では 4.0 m/s まで落ちない
（半径約 5.3 m 以下でないと 4.0 m/s にならない）。予測経路が曲がり始めた時点で上限を出し、
曲がる前に落としておく。
"""

from typing import Optional

import numpy as np


def turn_angle_deg(waypoints: np.ndarray) -> float:
    """経路の終わり（9→10 m 先の区間）の向き [deg]、左が正。

    右左折の正解率を測った評価 (eval_rgbmask/branch_eval.py) と同じく、最後の点と 2 つ前の点で取る
    """
    points = np.asarray(waypoints, dtype=np.float64).reshape(-1, 2)
    dx, dy = points[-1] - points[-3]
    return float(np.degrees(np.arctan2(dy, dx)))


class SpeedLimiter:
    def __init__(self, limit_mps: float = 4.0, turn_deg: float = 15.0, hold_s: float = 1.0):
        self.limit_mps = limit_mps
        self.turn_deg = turn_deg
        # 15° 付近で判定が行き来しても上限が出たり消えたりしないよう、最後に条件を満たしてから保持する
        self.hold_s = hold_s
        self._last_triggered_s: Optional[float] = None

    def update(self, waypoints: np.ndarray, pylon_near: bool, now_s: float) -> Optional[float]:
        """上限を出すべきなら limit_mps、出さないなら None"""
        if pylon_near or abs(turn_angle_deg(waypoints)) >= self.turn_deg:
            self._last_triggered_s = now_s
        if self._last_triggered_s is not None and now_s - self._last_triggered_s <= self.hold_s:
            return self.limit_mps
        return None
