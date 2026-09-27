import math
from collections import deque
from dataclasses import dataclass


@dataclass
class SteeringActuatorParams:
    dead_time: float = 0.04
    tau: float = 0.12
    gain: float = 0.9
    rate_max: float = math.radians(80.0)
    a_lat_max: float = 4.0
    wheel_base: float = 0.8
    stop_velocity: float = 0.1
    v_min: float = 0.5


class SteeringActuatorModel:
    def __init__(self, params: SteeringActuatorParams) -> None:
        self._params = params
        self._history = deque()
        self._held_cmd = 0.0
        self._rate_state = 0.0
        self._filt = 0.0
        self._t_prev = None

    def set_command(self, t: float, steering_cmd: float) -> None:
        self._history.append((t, steering_cmd))

    def update(self, t: float, velocity: float) -> float:
        p = self._params
        if self._t_prev is None:
            self._t_prev = t
            return self._filt
        dt = t - self._t_prev
        self._t_prev = t
        while self._history and self._history[0][0] <= t - p.dead_time:
            self._held_cmd = self._history.popleft()[1]
        target = 0.0 if abs(velocity) < p.stop_velocity else p.gain * self._held_cmd
        self._rate_state += max(-p.rate_max * dt, min(p.rate_max * dt, target - self._rate_state))
        self._filt += dt / (p.tau + dt) * (self._rate_state - self._filt)
        return self._filt

    def yaw_rate(self, velocity: float, steering: float) -> float:
        p = self._params
        w_max = p.a_lat_max / max(abs(velocity), p.v_min)
        return max(-w_max, min(w_max, velocity * math.tan(steering) / p.wheel_base))
