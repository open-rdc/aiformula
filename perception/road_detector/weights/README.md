## モデル変換

### `.pt` → `.onnx`
https://github.com/CAIC-AD/YOLOPv2 の `yolopv2.pt` をこのディレクトリに置いてください
```bash
python3 export_yolopv2_onnx.py \
  --height 384 \
  --width 640
```
`--height` は camera.size で決まる（nHD: 384、SVGA: 416）

### `.onnx` → `.engine`（デプロイ先で実行）
```bash
trtexec \
  --onnx=yolopv2.onnx \
  --saveEngine=yolopv2.engine \
  --fp16
```
