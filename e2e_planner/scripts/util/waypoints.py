#!/usr/bin/env python3
"""waypoint ラベルの生成ロジック。

オンライン収集 (create_data.py) とオフライン収集 (create_data_from_bag.py) の
両方から使う。片方だけ直すとラベルの定義がズレて、同じデータセットのつもりで
別物を学習させてしまうため、ここに集約する。

姿勢は ECEF 位置 + ヨー角で扱う。ヨーは ENU (東=x, 北=y) 基準。
"""

import numpy as np
import pymap3d as pm
from typing import List, Optional, Sequence, Tuple


class PoseSample:
    """補間に使う最小限の姿勢。ECEF 位置とヨー角だけ持つ"""

    def __init__(self, timestamp: float, position: np.ndarray, yaw: float):
        self.timestamp: float = timestamp
        self.position: np.ndarray = position
        self.yaw: float = yaw


def _interpolate(before: PoseSample, after: PoseSample, ratio: float, timestamp: float) -> PoseSample:
    position = before.position + (after.position - before.position) * ratio
    # ヨーは ±pi をまたぐので最短回転側で補間する
    delta_yaw = (after.yaw - before.yaw + np.pi) % (2.0 * np.pi) - np.pi
    yaw = before.yaw + delta_yaw * ratio
    return PoseSample(timestamp, position, yaw)


def interpolate_pose(pose_history: Sequence[PoseSample], target_time: float) -> Optional[PoseSample]:
    """target_time を挟む2点から線形補間した姿勢を返す。
    最近傍で済ませると pose レートの半周期分（常に未来寄り）のバイアスが乗るため補間する"""
    if len(pose_history) < 2:
        return None
    if target_time < pose_history[0].timestamp or target_time > pose_history[-1].timestamp:
        return None

    for i in range(len(pose_history) - 1):
        before = pose_history[i]
        after = pose_history[i + 1]
        if after.timestamp < target_time:
            continue

        span = after.timestamp - before.timestamp
        ratio = 0.0 if span <= 0.0 else (target_time - before.timestamp) / span
        return _interpolate(before, after, ratio, target_time)

    return None


def build_forward_arc(
    pose_history: Sequence[PoseSample], reference: PoseSample
) -> List[Tuple[float, PoseSample]]:
    """reference から前方の pose に累積走行距離を付けて返す [(累積距離[m], pose), ...]

    距離は ECEF 座標のユークリッド距離で積算する。数十 m の区間なら
    地表の移動距離との差は無視できる。
    """
    arc: List[Tuple[float, PoseSample]] = [(0.0, reference)]
    cumulative = 0.0
    previous = reference
    for pose in pose_history:
        if pose.timestamp <= reference.timestamp:
            continue
        cumulative += float(np.linalg.norm(pose.position - previous.position))
        arc.append((cumulative, pose))
        previous = pose
    return arc


def pose_at_distance(
    arc: Sequence[Tuple[float, PoseSample]], target_distance: float
) -> Optional[PoseSample]:
    """累積距離が target_distance になる点を線形補間で返す。まだ届いていなければ None"""
    if not arc or arc[-1][0] < target_distance:
        return None

    for i in range(1, len(arc)):
        before_distance, before = arc[i - 1]
        after_distance, after = arc[i]
        if after_distance < target_distance:
            continue

        span = after_distance - before_distance
        ratio = 0.0 if span <= 0.0 else (target_distance - before_distance) / span
        timestamp = before.timestamp + (after.timestamp - before.timestamp) * ratio
        return _interpolate(before, after, ratio, timestamp)

    return None


def transform_to_robot_frame(reference: PoseSample, current: PoseSample) -> Tuple[float, float]:
    """current を reference のロボット座標系 (x: 前方, y: 左) へ写す"""
    lat0, lon0, alt0 = pm.ecef2geodetic(reference.position[0], reference.position[1], reference.position[2])
    yaw0 = reference.yaw

    e, n, u = pm.ecef2enu(current.position[0], current.position[1], current.position[2], lat0, lon0, alt0)

    x_robot = -e * np.sin(yaw0) + n * np.cos(yaw0)
    y_robot = -e * np.cos(yaw0) - n * np.sin(yaw0)

    return x_robot, y_robot
