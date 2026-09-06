## モデル変換

モデル名は学習ごとに変わるため、以下の変数を実際の名前に読み替えてください。

```bash
EXP=<学習に使った exp ファイル名>
CKPT=<チェックポイント名>
MODEL=<出力するモデル名>
```

### `.pth` → `.onnx`
https://github.com/open-rdc/YOLOX を利用してください
```bash
python3 tools/export_onnx.py \
  -f exps/${EXP}.py \
  -c weights/${CKPT}.pth \
  --output-name ${MODEL}.onnx \
  --opset 11
```

### `.onnx` → `.engine`（デプロイ先で実行）
```bash
trtexec \
  --onnx=${MODEL}.onnx \
  --saveEngine=${MODEL}.engine \
  --fp16
```