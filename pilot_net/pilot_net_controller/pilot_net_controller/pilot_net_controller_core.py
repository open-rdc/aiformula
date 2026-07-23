import cv2
import numpy as np

from .model.pilotnet import PilotNetNp

IMAGE_HEIGHT = 288
IMAGE_WIDTH = 512


class PilotNetControllerCore:
    def __init__(self, weights_path: str, steering_max: float, velocity_max: float):
        self.model = PilotNetNp(weights_path)
        self.scale = np.array([steering_max, velocity_max], dtype=np.float32)

    def infer(self, bgr_image: np.ndarray) -> tuple[float, float]:
        yuv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2YUV)
        yuv = cv2.resize(yuv, (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_AREA)
        yuv = yuv.astype(np.float32) / 127.5 - 1.0
        x = np.transpose(yuv, (2, 0, 1))[np.newaxis, ...]

        output = self.model.forward(x)[0] * self.scale
        return float(output[0]), float(output[1])
