from typing import Tuple

import cv2
import numpy as np


PLACENET_CROP_SIZE = 288
MODEL_INPUT_SIZE = (85, 85)


def center_square_crop(image: np.ndarray, crop_size: int = PLACENET_CROP_SIZE) -> np.ndarray:
    height, width = image.shape[:2]
    if height < crop_size or width < crop_size:
        raise ValueError(f'Image is smaller than {crop_size}x{crop_size}: {width}x{height}')

    top = (height - crop_size) // 2
    left = (width - crop_size) // 2
    return image[top:top + crop_size, left:left + crop_size]


def color_mask_to_binary(mask_image: np.ndarray) -> np.ndarray:
    if mask_image.ndim == 2:
        return (mask_image > 0).astype(np.uint8)

    red_mask = (
        (mask_image[:, :, 2] > 200)
        & (mask_image[:, :, 0] < 50)
        & (mask_image[:, :, 1] < 50)
    )
    bright_mask = mask_image.max(axis=2) > 127
    return (red_mask | bright_mask).astype(np.uint8)


def preprocess_lane_mask(mask: np.ndarray) -> np.ndarray:
    binary_mask = color_mask_to_binary(mask)
    height, width = binary_mask.shape[:2]
    if height < PLACENET_CROP_SIZE or width < PLACENET_CROP_SIZE:
        return cv2.resize(binary_mask, MODEL_INPUT_SIZE, interpolation=cv2.INTER_NEAREST)

    cropped_mask = center_square_crop(binary_mask)
    return cv2.resize(cropped_mask, MODEL_INPUT_SIZE, interpolation=cv2.INTER_NEAREST)


def lane_mask_to_tensor_array(mask: np.ndarray) -> np.ndarray:
    return preprocess_lane_mask(mask).astype(np.float32)


def overlay_lane_mask(image_bgr: np.ndarray, processed_mask: np.ndarray) -> np.ndarray:
    cropped_image = center_square_crop(image_bgr)
    debug_image = cv2.resize(cropped_image, MODEL_INPUT_SIZE, interpolation=cv2.INTER_AREA)
    debug_image[processed_mask == 1] = [0, 0, 255]
    return debug_image
