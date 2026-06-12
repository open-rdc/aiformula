#!/usr/bin/env python3

import cv2
import numpy as np
import torch
from pathlib import Path
from typing import Optional, Tuple


class YOLOPv2Processor:
    def __init__(self, model_path: Path, device: torch.device, input_size: int = 640):
        self.device = device
        self.input_shape = (input_size, input_size)

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
        if ll.shape[1] == 1:
            ll_seg_mask = (ll[:, 0] > 0.5).int()
        else:
            ll_seg_mask = torch.argmax(ll, dim=1).int()
        return ll_seg_mask.squeeze().cpu().numpy()

    def _restore_original_size(
        self,
        mask: np.ndarray,
        original_shape: Tuple[int, int],
        ratio: float,
        pad: Tuple[float, float],
    ) -> np.ndarray:
        original_h, original_w = original_shape
        pad_left, pad_top = pad
        top = max(int(round(pad_top - 0.1)), 0)
        left = max(int(round(pad_left - 0.1)), 0)
        unpad_h = int(round(original_h * ratio))
        unpad_w = int(round(original_w * ratio))

        unpadded = mask[top:top + unpad_h, left:left + unpad_w]
        return cv2.resize(unpadded, (original_w, original_h), interpolation=cv2.INTER_NEAREST)

    def process_image(self, image: np.ndarray, target_size: Optional[Tuple[int, int]] = None) -> np.ndarray:
        original_shape = image.shape[:2]
        img_resized, ratio, (pad_left, pad_top) = self.letterbox(image, self.input_shape)

        img = img_resized.astype(np.float32) / 255.0
        img = torch.from_numpy(np.transpose(img, (2, 0, 1))).unsqueeze(0).to(self.device)

        with torch.no_grad():
            outputs = self.model(img)
            [pred, anchor_grid], seg, ll = outputs

        ll_seg_mask = self.lane_line_mask(ll)
        original_mask = self._restore_original_size(
            ll_seg_mask,
            original_shape,
            ratio,
            (pad_left, pad_top),
        )

        if target_size is None:
            return original_mask

        return cv2.resize(
            original_mask,
            target_size,
            interpolation=cv2.INTER_NEAREST
        )
