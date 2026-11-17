from dataclasses import dataclass
from typing import Optional

from rl.utitites.lane_kf import LostEvent
from rl.utitites.lane_observation import ObserverConfig

NONE = 0
LEFT_DEPARTURE = -1
RIGHT_DEPARTURE = 1
ANOMALY = 2


@dataclass
class DepartureConfig:
    min_track_time: float = 0.005
    edge_margin: float = 30.0


class DepartureDetector:
    def __init__(self, config: DepartureConfig, observer_config: ObserverConfig):
        self.cfg = config
        self.obs_cfg = observer_config

    def bottom_x(self, event: LostEvent, height: int) -> float:
        y_bottom = min(self.obs_cfg.y_end, height) - 1
        x_ref, slope = event.state
        return float(x_ref + slope * (y_bottom - self.obs_cfg.y_ref))

    def check(self, left: Optional[LostEvent], right: Optional[LostEvent], width: int, height: int) -> int:
        if any(e is not None and e.reason == 'dt' for e in (left, right)):
            return ANOMALY
        left = left if self._long_lost(left) else None
        right = right if self._long_lost(right) else None
        if left is not None and right is not None:
            return ANOMALY
        m = self.cfg.edge_margin
        if left is not None and self.bottom_x(left, height) > m:
            print("left departure")
            return LEFT_DEPARTURE
        if right is not None and self.bottom_x(right, height) < width - 1 - m:
            print("right departure")
            return RIGHT_DEPARTURE
        return NONE

    def _long_lost(self, event: Optional[LostEvent]) -> bool:
        return event is not None and event.reason == 'lost' and event.track_time >= self.cfg.min_track_time
