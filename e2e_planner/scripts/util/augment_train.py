#!/usr/bin/env python3
"""学習時のみ適用するデータ拡張。

preprocess.py は学習と推論で共有するが、こちらは**学習側だけ**に入れる。
推論に混ぜると入力分布がずれるので inference_node からは import しないこと。

torchvision が環境に入っていないため OpenCV / numpy で実装している。
"""

import cv2
import numpy as np
from typing import Tuple


def horizontal_flip(
    bgr_image: np.ndarray,
    mask_binary: np.ndarray,
    waypoints_m: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """画像・マスクを左右反転し、waypoint の横方向を符号反転する。

    waypoints_m は正規化前の (N, 2) [m]。create_data.py の
    transform_to_robot_frame が x=前方 / y=横 で吐くので、左右反転に対応するのは
    y の符号反転だけ。**normalize_waypoints の前に**適用すること
    （正規化後は Y_OFFSET のぶんだけ原点がずれていて単純な符号反転にならない）。
    """
    flipped_wp = np.asarray(waypoints_m, dtype=np.float32).copy()
    flipped_wp[:, 1] = -flipped_wp[:, 1]
    return cv2.flip(bgr_image, 1), cv2.flip(mask_binary, 1), flipped_wp


def to_grayscale(bgr_image: np.ndarray) -> np.ndarray:
    """色を完全に落とす（3ch のまま）。

    色をショートカットに使わせないための拡張。シミュレータのコーンは色が
    位置と結びつきやすく、モデルが白線の形状ではなく色で分岐を判断してしまう
    （2026-09-05 に実際に発生）。確率的にグレースケールを混ぜると、
    色が無くても走れる特徴を学ばざるを得なくなる。
    """
    gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def color_jitter(
    bgr_image: np.ndarray,
    rng: np.random.Generator,
    brightness: float = 0.3,
    contrast: float = 0.3,
    saturation: float = 0.3,
    hue: float = 0.03,
) -> np.ndarray:
    """明度・コントラスト・彩度・色相をランダムに変える。

    マスクには**適用しない**。mask_images は赤(0,0,255)で描かれていて
    extract_red_mask が R>200 かつ B<50 かつ G<50 で拾っているため、
    色をいじると判定が壊れる。
    """
    out = bgr_image.astype(np.float32)

    if brightness > 0:
        out *= rng.uniform(1.0 - brightness, 1.0 + brightness)

    if contrast > 0:
        # グレースケール平均まわりに伸縮する
        mean = float(cv2.cvtColor(np.clip(out, 0, 255).astype(np.uint8),
                                  cv2.COLOR_BGR2GRAY).mean())
        out = (out - mean) * rng.uniform(1.0 - contrast, 1.0 + contrast) + mean

    out = np.clip(out, 0, 255).astype(np.uint8)

    if saturation > 0 or hue > 0:
        hsv = cv2.cvtColor(out, cv2.COLOR_BGR2HSV).astype(np.float32)
        if saturation > 0:
            hsv[:, :, 1] = np.clip(
                hsv[:, :, 1] * rng.uniform(1.0 - saturation, 1.0 + saturation), 0, 255)
        if hue > 0:
            # OpenCV の H は 0-179。剰余で巻き戻す
            hsv[:, :, 0] = (hsv[:, :, 0] + rng.uniform(-hue, hue) * 180.0) % 180.0
        out = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

    return out
