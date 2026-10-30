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

    def lane_line_mask(self, ll: torch.Tensor, target_shape: Tuple[int, int]) -> np.ndarray:
        """車線マスクを letterbox 後の入力解像度に揃えて 0/1 の numpy 配列で返す。

        YOLOPv2 のオリジナル実装は ll が入力の 1/2 解像度で出てくる前提で
        scale_factor=2 を掛けているが、この TorchScript モデルは入力と同解像度で
        出力する。以降でパディングを剥がす座標計算が入力解像度基準なので、
        ここで必ず入力解像度へ揃える。
        """
        if ll.shape[-2:] != target_shape:
            ll = torch.nn.functional.interpolate(
                ll, size=target_shape, mode='bilinear', align_corners=False
            )
        ll_seg_mask = torch.round(ll).squeeze(1)
        return ll_seg_mask.int().squeeze().cpu().numpy()

    def process_image(self, image: np.ndarray, target_size: Tuple[int, int]) -> np.ndarray:
        height, width = image.shape[:2]
        img_resized, ratio, (pad_left, pad_top) = self.letterbox(image, self.input_shape)

        img = img_resized.astype(np.float32) / 255.0
        img = torch.from_numpy(np.transpose(img, (2, 0, 1))).unsqueeze(0).to(self.device)

        with torch.no_grad():
            outputs = self.model(img)
            [pred, anchor_grid], seg, ll = outputs

        ll_seg_mask = self.lane_line_mask(ll, img_resized.shape[:2])

        # letterbox で足した灰色パディングを剥がしてから元画像の座標系へ戻す。
        # 剥がさないとパディング分だけマスクが RGB に対してスケール/オフセットずれを起こす
        top, left = int(round(pad_top - 0.1)), int(round(pad_left - 0.1))
        unpad_h, unpad_w = int(round(height * ratio)), int(round(width * ratio))
        ll_seg_mask = ll_seg_mask[top:top + unpad_h, left:left + unpad_w]

        if (ll_seg_mask.shape[1], ll_seg_mask.shape[0]) == target_size:
            return ll_seg_mask.astype(np.uint8)

        resized_mask = cv2.resize(
            ll_seg_mask.astype(np.uint8),
            target_size,
            interpolation=cv2.INTER_NEAREST
        )

        return resized_mask
