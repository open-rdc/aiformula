"""走った距離で過去フレームを引くためのバッファ。

学習 (train_curve_weight.py の _past_stems) は、各サンプルの累積走行距離 arc_m から
「d m 前に一番近いフレーム」を選び、その距離ぶん戻れない走り始めは最古のフレームで代用する。
推論でも同じ選び方をする。

重くならないように:
- 貯めるのは、各フレームで一度だけ計算した前処理済みの入力 (256x144 の RGB と過去マスク)。
  過去フレームとして使うときに YOLOPv2 / YOLOX を回し直さない
- 前回貯めた位置から spacing_m 以上進んだときだけ貯める。止まっている間は増えない
- 一番遠い必要距離 + spacing_m より古いものは捨てる。
  長さは max(distances) / spacing_m + 2 程度で頭打ちになる (3.0 m / 0.25 m なら 14 枚)
"""

from collections import deque
from typing import Any, Deque, List, Optional, Sequence, Tuple


class DistanceFrameHistory:
    def __init__(self, distances_m: Sequence[float], spacing_m: float = 0.25) -> None:
        if spacing_m <= 0.0:
            raise ValueError(f'spacing_m は正の値: {spacing_m}')
        self.max_distance_m = max(distances_m) if distances_m else 0.0
        self.spacing_m = spacing_m
        # (arc_m, item) を古い順に持つ
        self.frames: Deque[Tuple[float, Any]] = deque()

    def __len__(self) -> int:
        return len(self.frames)

    def clear(self) -> None:
        self.frames.clear()

    def push(self, arc_m: float, item: Any) -> bool:
        """今のフレームを貯める。前回から spacing_m 進んでいなければ貯めない。貯めたら True"""
        if self.max_distance_m <= 0.0:
            return False
        if self.frames and arc_m - self.frames[-1][0] < self.spacing_m:
            return False
        self.frames.append((arc_m, item))
        # 一番遠い過去 (max_distance_m 前) を選ぶのに要る 1 枚は残し、それより古いものを捨てる
        keep_from = arc_m - self.max_distance_m - self.spacing_m
        while len(self.frames) >= 2 and self.frames[1][0] <= keep_from:
            self.frames.popleft()
        return True

    def lookup(self, arc_now: float, current: Any, distances_m: Sequence[float]) -> List[Any]:
        """distances_m の各距離だけ前に一番近いフレームの item。

        候補は貯めたフレームと今のフレーム (学習も今のフレーム自身を候補に含む)。
        その距離ぶん戻れないときは最古のフレームで代用する (学習と同じ)
        """
        candidates: List[Tuple[float, Any]] = list(self.frames)
        if not candidates or candidates[-1][0] < arc_now:
            candidates.append((arc_now, current))
        result = []
        for distance in distances_m:
            target = arc_now - distance
            if target <= candidates[0][0]:
                result.append(candidates[0][1])
                continue
            best = min(candidates, key=lambda c: abs(c[0] - target))
            result.append(best[1])
        return result


class Odometer:
    """速さ [m/s] を時間で足して累積走行距離 [m] を出す。

    学習の arc_m (create_data_from_bag.py) と同じく vectornav の速度から出す
    (/vectornav/velocity_body の body 系の速さ)。間が空いたときは足さない
    """

    def __init__(self, max_gap_s: float = 0.5) -> None:
        self.max_gap_s = max_gap_s
        self.arc_m = 0.0
        self.last_time_s: Optional[float] = None

    def update(self, speed_mps: float, time_s: float) -> None:
        if self.last_time_s is not None:
            dt = time_s - self.last_time_s
            if 0.0 < dt <= self.max_gap_s:
                self.arc_m += abs(speed_mps) * dt
        self.last_time_s = time_s

    def is_fresh(self, now_s: float) -> bool:
        return self.last_time_s is not None and 0.0 <= now_s - self.last_time_s <= self.max_gap_s
