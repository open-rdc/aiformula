## モデル変換

### `.pth` → `.onnx`
https://github.com/open-rdc/YOLOX を利用してください
```bash
python3 tools/export_onnx.py \
  -f exps/yolox_s_cone_nhd.py \
  -c weights/yolox_s_merged_nhd_best_ckpt.pth \
  --output-name yolox_s_merged_nhd.onnx \
  --opset 11
```

### `.onnx` → `.engine`（デプロイ先で実行）
```bash
trtexec \
  --onnx=yolox_s_merged_nhd.onnx \
  --saveEngine=yolox_s_merged_nhd.engine \
  --fp16
```
