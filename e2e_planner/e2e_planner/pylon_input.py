"""推論に使うパイロンマスクを選ぶ。

マスクは deploy の pylon_detector が別ノードで出すので、未着・遅れ・解像度違いがありうる。
そのときも経路は止めずに、パイロンが無いとみなした 0 埋めのマスクで推論を続ける。
"""

from typing import Optional

import cv2
import numpy as np

MASK_HEIGHT = 360
MASK_WIDTH = 640
MAX_AGE_S = 0.5


def fresh_pylon_mask(mask: Optional[np.ndarray], age_s: Optional[float],
                     max_age_s: float = MAX_AGE_S) -> np.ndarray:
    """age_s 秒前に届いた mask を 640x360 の uint8 で返す。無いか古ければ 0 埋め"""
    if mask is None or age_s is None or age_s > max_age_s:
        return np.zeros((MASK_HEIGHT, MASK_WIDTH), np.uint8)
    if mask.shape[:2] != (MASK_HEIGHT, MASK_WIDTH):
        mask = cv2.resize(mask, (MASK_WIDTH, MASK_HEIGHT), interpolation=cv2.INTER_NEAREST)
    return mask
