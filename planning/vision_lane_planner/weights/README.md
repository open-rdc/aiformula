## モデル変換

モデル名は学習ごとに変わるため、以下の変数を実際の名前に読み替えてください。

```bash
CKPT=<学習済みチェックポイント (.pt)>
MODEL=<出力するモデル名>
```

### `.pt` → `.onnx`（学習環境で実行）

`learning/` を作業ディレクトリにして実行します。

```bash
python3 -m Models.deploy.export_onnx \
  --weights ${CKPT} \
  --output  ${MODEL}.onnx
```

`head='argmax'` のチェックポイントのみ対応します（`dist` は `ValueError` で中断します）。
出力名は `exist` / `valid` / `position`、入力は `(1,3,384,640)` 固定です。

### `.onnx` → `.engine`（デプロイ先で実行）

```bash
trtexec \
  --onnx=${MODEL}.onnx \
  --saveEngine=${MODEL}.engine \
  --fp16
```

engine は実機・TensorRT バージョンごとに再生成が必要です。
生成した `.engine` をこのディレクトリに置き、`main_params.yaml` の
`vision_lane_planner_node.engine_path` にファイル名を書きます。
