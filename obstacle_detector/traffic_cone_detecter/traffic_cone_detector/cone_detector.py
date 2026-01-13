"""
Traffic Cone Detector Model Library
"""

import torch
import os


class ConeDetector:
    """YOLOv5-based traffic cone detector"""

    def __init__(self, weights_path='weights/best.pt', conf_thres=0.25, iou_thres=0.45):
        """
        Initialize cone detector

        Args:
            weights_path: Path to model weights
            conf_thres: Confidence threshold
            iou_thres: IOU threshold for NMS
        """
        # Disable YOLOv5 auto-install to prevent pip dependency errors
        os.environ['YOLOv5_AUTOINSTALL'] = 'False'
        os.environ['YOLOv5_VERBOSE'] = 'False'

        # Use local YOLOv5 repository to avoid dependency issues
        yolov5_path = '/home/nvidia/yolov5'
        if os.path.exists(yolov5_path):
            self.model = torch.hub.load(yolov5_path, 'custom', path=weights_path, source='local')
        else:
            # Fallback to online loading if local repo not available
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=weights_path)

        self.model.conf = conf_thres
        self.model.iou = iou_thres

    def detect(self, frame):
        """
        Run detection on a frame

        Args:
            frame: Input image (numpy array)

        Returns:
            results: YOLOv5 results object
        """
        return self.model(frame)

    def get_detections(self, results):
        """
        Get detection details as pandas DataFrame

        Args:
            results: YOLOv5 results object

        Returns:
            DataFrame with columns: xmin, ymin, xmax, ymax, confidence, class, name
        """
        return results.pandas().xyxy[0]

    def render(self, results):
        """
        Render detection results on frame

        Args:
            results: YOLOv5 results object

        Returns:
            Annotated frame (numpy array)
        """
        return results.render()[0].copy()
