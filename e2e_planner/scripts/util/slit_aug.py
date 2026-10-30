import numpy as np
from typing import List, Tuple


def crop_images(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    center_crop = image[:, 40:440]
    right_crop = image[:, 80:480]
    left_crop = image[:, 0:400]
    return center_crop, right_crop, left_crop


def rotate_waypoints(waypoints: List[List[float]], angle: float) -> List[List[float]]:
    cos_theta = np.cos(angle)
    sin_theta = np.sin(angle)
    rotation_matrix = np.array([[cos_theta, -sin_theta],
                                [sin_theta, cos_theta]])

    rotated_waypoints = []
    for waypoint in waypoints:
        rotated = rotation_matrix @ np.array(waypoint)
        rotated_waypoints.append(rotated.tolist())

    return rotated_waypoints


def augment(image: np.ndarray, waypoints: List[List[float]]) -> List[Tuple[np.ndarray, List[List[float]]]]:
    center_crop, right_crop, left_crop = crop_images(image)

    center_waypoints = waypoints
    right_waypoints = rotate_waypoints(waypoints, 0.1745)
    left_waypoints = rotate_waypoints(waypoints, -0.1745)

    return [
        (center_crop, center_waypoints),
        (right_crop, right_waypoints),
        (left_crop, left_waypoints)
    ]
