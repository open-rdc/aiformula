#!/usr/bin/env python3
"""パイロンの枠を塗りつぶした 1ch マスクを作る。

学習データ（オフライン、ここの YoloxOnnx）と実機（deploy の pylon_detector が
TensorRT で出す /perception/pylon_mask）で同じものにするため、検出の前処理・デコード・NMS は
perception/object_detection/src/yolox_tensorrt.cpp を、塗り方は同じく pylon_mask.cpp を写している。
どちらかを変えたら両方を直すこと。
"""

import cv2
import numpy as np

PYLON_SCORE_THRESHOLD = 0.3
PYLON_NMS_THRESHOLD = 0.45
PYLON_CLASS_ID = 0

MASK_HEIGHT = 360
MASK_WIDTH = 640

# 「コーンが近い（自車線の前方にある）」とみなす範囲。640x360 の画素座標。
# 9/22 の障害物 bag で、コーンの横を通る直前のフレームの枠の下端と中心がこの範囲に入っていた
NEAR_MIN_ROW = 215
NEAR_MIN_COL = 120
NEAR_MAX_COL = 520

STRIDES = (8, 16, 32)
LETTERBOX_VALUE = 114


class YoloxOnnx:
    """YOLOX の ONNX（デコード前の出力 (1, anchors, 5 + classes)）を onnxruntime の CPU で回す"""

    def __init__(self, onnx_path: str):
        import onnxruntime

        self.session = onnxruntime.InferenceSession(onnx_path, providers=['CPUExecutionProvider'])
        model_input = self.session.get_inputs()[0]
        self.input_name = model_input.name
        self.input_height, self.input_width = model_input.shape[2], model_input.shape[3]

    def _letterbox(self, bgr: np.ndarray):
        """アスペクト比を保って縮小し左上詰めで 114 埋め、正規化はしない"""
        ratio = min(self.input_height / bgr.shape[0], self.input_width / bgr.shape[1])
        resized = cv2.resize(bgr, (int(bgr.shape[1] * ratio), int(bgr.shape[0] * ratio)),
                             interpolation=cv2.INTER_LINEAR)
        padded = np.full((self.input_height, self.input_width, 3), LETTERBOX_VALUE, np.uint8)
        padded[:resized.shape[0], :resized.shape[1]] = resized
        return padded, ratio

    def _decode(self, output: np.ndarray):
        grids, strides = [], []
        for stride in STRIDES:
            grid_y, grid_x = np.meshgrid(np.arange(self.input_height // stride),
                                         np.arange(self.input_width // stride), indexing='ij')
            grids.append(np.stack([grid_x.ravel(), grid_y.ravel()], axis=1))
            strides.append(np.full(grid_x.size, stride))
        grid = np.concatenate(grids).astype(np.float64)
        stride = np.concatenate(strides).astype(np.float64)[:, None]

        class_scores = output[:, 5:]
        class_ids = class_scores.argmax(axis=1)
        scores = output[:, 4] * class_scores.max(axis=1)
        keep = scores >= PYLON_SCORE_THRESHOLD

        centers = (output[keep, :2] + grid[keep]) * stride[keep]
        sizes = np.exp(output[keep, 2:4]) * stride[keep]
        boxes = np.concatenate([centers - sizes * 0.5, sizes], axis=1)  # x, y, w, h
        return boxes, scores[keep], class_ids[keep]

    def detect(self, bgr: np.ndarray) -> np.ndarray:
        """(N, 6) = x1, y1, x2, y2, score, class_id（元画像の画素座標）"""
        padded, ratio = self._letterbox(bgr)
        blob = padded.transpose(2, 0, 1)[None].astype(np.float32)
        output = self.session.run(None, {self.input_name: blob})[0][0]

        boxes, scores, class_ids = self._decode(output)
        keep = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), PYLON_SCORE_THRESHOLD, PYLON_NMS_THRESHOLD)
        keep = np.asarray(keep, int).reshape(-1)

        detections = np.zeros((len(keep), 6), np.float32)
        for row, index in enumerate(keep):
            x, y, w, h = boxes[index] / ratio
            detections[row] = (x, y, x + w, y + h, scores[index], class_ids[index])
        return detections


def draw_pylon_mask(detections: np.ndarray, height: int = MASK_HEIGHT, width: int = MASK_WIDTH) -> np.ndarray:
    """pylon かつ score >= 0.3 の枠を 255 で塗った uint8 マスク。

    座標は C++ 側の cv::Rect(cv::Rect2d) 変換（saturate_cast = cvRound、偶数丸め）と同じく
    x, y, 幅, 高さをそれぞれ丸める
    """
    mask = np.zeros((height, width), np.uint8)
    for x1, y1, x2, y2, score, class_id in np.asarray(detections, np.float32).reshape(-1, 6):
        if int(class_id) != PYLON_CLASS_ID or score < PYLON_SCORE_THRESHOLD:
            continue
        left, top = round(float(x1)), round(float(y1))
        right, bottom = left + round(float(x2 - x1)), top + round(float(y2 - y1))
        cv2.rectangle(mask, (left, top), (right - 1, bottom - 1), 255, cv2.FILLED)
    return mask


def is_pylon_near(mask: np.ndarray) -> bool:
    """640x360 の 0/255 マスクで、前方の近い範囲にパイロンの塗りがあるか"""
    return bool(mask[NEAR_MIN_ROW:, NEAR_MIN_COL:NEAR_MAX_COL + 1].any())
