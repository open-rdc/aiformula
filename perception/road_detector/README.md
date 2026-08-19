## モデル変換

### `.pt` → `.onnx`
https://github.com/CAIC-AD/YOLOPv2 の `yolopv2.pt` を `data/weights/` に置いてください
```bash
python3 scripts/export_yolopv2_onnx.py \
  --weights perception/road_detector/data/weights/yolopv2.pt \
  --output perception/road_detector/data/weights/yolopv2.onnx \
  --height 384 --width 640
```
`--height` は camera.size で決まる（nHD: 384、SVGA: 416）

### `.onnx` → `.engine`（デプロイ先で実行）
```bash
trtexec \
  --onnx=perception/road_detector/data/weights/yolopv2.onnx \
  --saveEngine=perception/road_detector/data/weights/yolopv2.engine \
  --fp16
```
