from dataclasses import dataclass
from typing import Optional

import numpy as np

from rl.utitites.lane_observation import LineObservation, LinePrior


@dataclass
class KalmanConfig:
    q_x: float = 900.0
    q_slope: float = 0.5
    gate_chi2: float = 13.8
    max_lost_frames: int = 10
    max_sigma_x: float = 60.0
    max_dt: float = 0.5


@dataclass
class LostEvent:
    reason: str
    state: np.ndarray
    track_time: float


class LineKalmanFilter:
    def __init__(self, config: KalmanConfig = KalmanConfig()):
        self.cfg = config
        self.event: Optional[LostEvent] = None
        self.s = None
        self.reset()

    def reset(self, reason: Optional[str] = None):
        if reason is not None and self.initialized:
            self.event = LostEvent(reason, self.s.copy(), self.age_at_update)
        self.s: Optional[np.ndarray] = None
        self.P: Optional[np.ndarray] = None
        self.lost_frames = 0
        self.age = 0.0
        self.age_at_update = 0.0

    def pop_event(self) -> Optional[LostEvent]:
        event, self.event = self.event, None
        return event

    @property
    def initialized(self) -> bool:
        return self.s is not None

    def predict(self, dt: float):
        if not self.initialized:
            return
        if dt < 0.0 or dt > self.cfg.max_dt:
            self.reset('dt')
            return
        self.age += dt
        self.P = self.P + np.diag([self.cfg.q_x, self.cfg.q_slope]) * dt
        if np.sqrt(self.P[0, 0]) > self.cfg.max_sigma_x:
            self.reset('diverged')

    def update(self, obs: LineObservation) -> bool:
        if not obs.valid:
            self._miss()
            return False
        z = np.array([obs.x_ref, obs.slope])
        R = np.diag([obs.var_x, obs.var_slope])
        if not self.initialized:
            self.s, self.P = z, R
            self.lost_frames = 0
            return True

        innovation = z - self.s
        S = self.P + R
        if innovation @ np.linalg.solve(S, innovation) > self.cfg.gate_chi2:
            self._miss()
            return False
        K = self.P @ np.linalg.inv(S)
        self.s = self.s + K @ innovation
        I_K = np.eye(2) - K
        self.P = I_K @ self.P @ I_K.T + K @ R @ K.T
        self.lost_frames = 0
        self.age_at_update = self.age
        return True

    def _miss(self):
        if not self.initialized:
            return
        self.lost_frames += 1
        if self.lost_frames > self.cfg.max_lost_frames:
            self.reset('lost')

    def prior(self) -> Optional[LinePrior]:
        if not self.initialized:
            return None
        return LinePrior(float(self.s[0]), float(self.s[1]), float(np.sqrt(self.P[0, 0])))

    def to_list(self):
        if not self.initialized:
            return [0.0, float('nan'), float('nan'), float('inf'), float('inf')]
        return [1.0, float(self.s[0]), float(self.s[1]), float(self.P[0, 0]), float(self.P[1, 1])]
