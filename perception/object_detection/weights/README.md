## モデル変換

ONNX をデプロイ先で TensorRT engine に変換して使います。

### `.onnx` → `.engine`（デプロイ先で実行）
```bash
trtexec \
  --onnx=rfdetr_l_aiformula.onnx \
  --saveEngine=rfdetr_l_aiformula.engine \
  --fp16
```