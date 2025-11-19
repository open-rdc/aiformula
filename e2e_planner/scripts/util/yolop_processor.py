#!/usr/bin/env python3

import cv2
import numpy as np
import torch
from pathlib import Path
from typing import Tuple


class YOLOPv2Processor:
    def __init__(self, model_path: Path, device: torch.device):
        self.device = device
        self.input_shape = (640, 640)

        if model_path.exists():
            self.model = torch.jit.load(str(model_path), map_location=device)
            self.model.to(device)
            self.model.eval()
        else:
            raise FileNotFoundError(f'YOLOPv2 model not found: {model_path}')

    def letterbox(self, img: np.ndarray, new_shape: Tuple[int, int], color: Tuple[int, int, int] = (114, 114, 114), stride: int = 32) -> Tuple[np.ndarray, float, Tuple[float, float]]:
        shape = img.shape[:2]
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
        dw, dh = np.mod(dw, stride) / 2, np.mod(dh, stride) / 2

        if shape[::-1] != new_unpad:
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)

        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))

        img = cv2.copyMakeBorder(
            img, top, bottom, left, right,
            cv2.BORDER_CONSTANT, value=color
        )

        return img, r, (dw, dh)

    def lane_line_mask(self, ll: torch.Tensor) -> np.ndarray:
        ll_softmax = torch.nn.functional.softmax(ll, dim=1)
        ll_prob = ll_softmax[0, 1, :, :]
        mask = (ll_prob > 0.5).cpu().numpy().astype(np.uint8)
        return mask

    def process_image(self, image: np.ndarray, target_size: Tuple[int, int]) -> np.ndarray:
        img_resized, ratio, (pad_left, pad_top) = self.letterbox(image, self.input_shape)

        img_normalized = img_resized.astype(np.float32) / 255.0
        img_tensor = torch.from_numpy(np.transpose(img_normalized, (2, 0, 1))).unsqueeze(0).to(self.device)

        with torch.no_grad():
            outputs = self.model(img_tensor)
            [pred, anchor_grid], seg, ll = outputs

        ll_seg_mask = self.lane_line_mask(ll)

        resized_mask = cv2.resize(
            ll_seg_mask,
            target_size,
            interpolation=cv2.INTER_NEAREST
        )

        return resized_mask
