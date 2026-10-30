#!/usr/bin/env python3
"""学習(train.py)と推論(inference_node)で共有する前処理。
どちらか一方だけを変更するとマスク/RGBの見え方がズレるため、ここに集約する。"""

import cv2
import numpy as np

# ZED HD720 の 1/2 (640x360, 16:9) をそのまま使う。左右の切り出しは行わない。
# 保存画像 640x360 の 1/2.5。アスペクト比 16:9 は厳密に保たれる (整数比ではないが
# INTER_AREA / INTER_NEAREST でそのまま縮小できる)。
# ConvStem が stride 2 を 3 回かけるため縦横とも 8 の倍数であること
# (1/2 の 320x180 は 180/8 が整数にならないので不可。640x360 からの整数比かつ
#  8 の倍数は 128x72 と 640x360 しか存在しない)。
# 128x72 から上げた値。トークン数 289 -> 1153 で遠方の白線が潰れにくくなる。
# 推論は 2.04ms -> 3.15ms (RTX 2070 Max-Q, batch=1) で、YOLOPv2 の 23ms に対し誤差の範囲。
# ここを変えると pos_embed の形が変わるため学習済みの重みは互換性を失う
IMAGE_WIDTH = 256
IMAGE_HEIGHT = 144

# waypoint の正規化定数。x は [0, 2*X_SCALE]、y は [-Y_OFFSET, 2*Y_SCALE - Y_OFFSET] を
# [-1, 1] に写す。Y_SCALE != Y_OFFSET にすると写像がゼロ中心にならないので必ず等しくすること。
#
# create_data.py の WAYPOINT_MODE = 'distance' (0.5 m x 20点 = 10 m 先まで) に合わせた値。
# 実機データ real20260927_v2 で実際に学習に使う 9277 サンプル (185540点) の実測は
# 0.458 <= x <= 9.991 m, -7.059 <= y <= 7.364 m (2026-09-27)。
# (ヨーの較正がずれていた 3 本を除いた後の値。除く前の 11335 サンプルでも
#  y の最大は 7.503 m でほぼ同じだった)
# x は 0.5 m x 20点 の定義から 10 m で頭打ちになるので X_SCALE = 5.0 で確定。
# y は 7.0 だと実機の最大旋回 (7.5 m) がレンジをはみ出すので 7.5 にした。
# これで正規化後の使用率は x 96%, y 97%。
# レンジが広すぎると出力の一部しか使われず MSE の効きが落ちる = 横位置のズレが罰せられにくい。
# 逆に狭すぎると外れ値だけが極端に大きな損失を出して学習が引っ張られる。
# ホライズンを変えたら、データセットを変えたらここは必ず測り直すこと
# (create_data_from_bag.py が実行の最後に実測レンジを出す)。
# ここを変えると学習済みの重みは互換性を失う
# (シムで学習した 20260910 系の重み (Y=7.0) とは非互換)
WAYPOINT_X_SCALE = 5.0
WAYPOINT_Y_SCALE = 7.5
WAYPOINT_Y_OFFSET = 7.5


def to_bgr(image: np.ndarray) -> np.ndarray:
    """BGRA / グレースケール / BGR のいずれでも 3ch BGR に揃える"""
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image


def extract_red_mask(bgr_image: np.ndarray) -> np.ndarray:
    """赤で描かれた白線マスク画像（mask_images / シミュレータ画像）を 0/1 に変換"""
    return ((bgr_image[:, :, 2] > 200) & (bgr_image[:, :, 0] < 50) & (bgr_image[:, :, 1] < 50)).astype(np.uint8)


def preprocess_mask(mask_binary: np.ndarray) -> np.ndarray:
    """0/1 マスク -> (1, H, W) float32"""
    resized = cv2.resize(mask_binary, (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_NEAREST)
    return resized.astype(np.float32)[np.newaxis, :, :]


def preprocess_rgb(bgr_image: np.ndarray) -> np.ndarray:
    """BGR 画像 -> (3, H, W) float32, [-1, 1] 正規化"""
    resized = cv2.resize(bgr_image, (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_AREA)
    normalized = resized.astype(np.float32) / 127.5 - 1.0
    return np.transpose(normalized, (2, 0, 1))


def resize_for_debug(bgr_image: np.ndarray) -> np.ndarray:
    """デバッグ表示用に前処理と同じサイズへ揃えた uint8 BGR"""
    return cv2.resize(bgr_image, (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_AREA)


def normalize_waypoints(waypoints: np.ndarray) -> np.ndarray:
    """(N, 2) [m] -> flatten して [-1, 1] 付近へ正規化"""
    flat = np.asarray(waypoints, dtype=np.float32).flatten()
    flat[0::2] = flat[0::2] / WAYPOINT_X_SCALE - 1.0
    flat[1::2] = (flat[1::2] + WAYPOINT_Y_OFFSET) / WAYPOINT_Y_SCALE - 1.0
    return flat


def denormalize_waypoints(normalized: np.ndarray) -> np.ndarray:
    """normalize_waypoints の逆変換"""
    denormalized = np.asarray(normalized, dtype=np.float32).copy()
    denormalized[0::2] = (normalized[0::2] + 1.0) * WAYPOINT_X_SCALE
    denormalized[1::2] = (normalized[1::2] + 1.0) * WAYPOINT_Y_SCALE - WAYPOINT_Y_OFFSET
    return denormalized
