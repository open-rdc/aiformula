# Traffic Cone Detection

YOLOv5ベースのパイロン検出システム

## 概要

このプロジェクトは、YOLOv5モデルを使用して、4色（赤、黄、緑、青）のトラフィックコーンをリアルタイムで検出します。

## モデル情報

- **ベースモデル**: YOLOv5
- **学習データ**: 4色のパイロン（赤、黄、緑、青）
- **重みファイル**: `weights/best.pt`
- **参考リポジトリ**: https://github.com/jhan15/traffic_cones_detection

## セットアップ

```bash
pip install -r requirements.txt
```

## 使用方法

### カメラからのリアルタイム検出

```bash
python3 detect_camera.py
```

### ライブラリとして使用

```python
from cone_detector import ConeDetector
import cv2

# 検出器の初期化
detector = ConeDetector()

# 画像から検出
image = cv2.imread('image.jpg')
results = detector.detect(image)

# 結果を描画
annotated = detector.render(results)

# 検出詳細を取得
detections = detector.get_detections(results)
print(f"Found {len(detections)} cones")
```