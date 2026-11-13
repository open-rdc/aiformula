## モデル変換

モデル名は学習ごとに変わるため、以下の変数を実際の名前に読み替えてください。

```bash
CKPT=<学習済みチェックポイント (.pt)>
MODEL=<出力するモデル名>
```

### `.pt` → `.onnx`（学習環境で実行）

`learning/` を作業ディレクトリにして実行します。

```bash
python3 -m Models.vector_planner.deploy.export \
  --checkpoint ${CKPT} \
  --output     ${MODEL}.onnx
```

入力は `image`（形状 `(1,3,384,640)` 固定、BGR 順・0..1）です。
ノードが読む出力は `branch_exist` `path_xy` `path_valid` `corridor_left` `corridor_right`
`signal_prob` `signal_xy` です（残りの地図・物体検出の出力は読みません）。
信号ヘッド付き（`--signal-head` で学習した）チェックポイントのみ出力できます。

### `.onnx` → `.engine`（デプロイ先で実行）

```bash
trtexec \
  --onnx=${MODEL}.onnx \
  --saveEngine=${MODEL}.engine \
  --fp16
```

engine は実機・TensorRT バージョンごとに再生成が必要です。
生成した `.engine` をこのディレクトリに置き、`main_params.yaml` の
`vector_e2e_planner_node.engine_path` にファイル名を書きます。
