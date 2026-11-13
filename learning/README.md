# learning

`learning/` を作業ディレクトリにして実行してください。

### rosbag → 教師データ
```bash
python3 -m Models.data_parsing.rosbag.converter \
  --bags ${BAG} \
  --out ${DATASET}
```

### 学習
```bash
python3 -m Models.vision_planner.training.vision_planner_trainer \
  --dataset ${DATASET} \
  --epochs 60
```

### 推論結果の可視化
```bash
python3 -m Models.vision_planner.viz.vision_planner_visualization \
  --bag ${BAG} \
  --weights runs/lane/run1/weights/best.pt \
  --out-video ${VIDEO} \
  --with-gt
```

### vector_planner
```bash
python3 -m Models.vector_planner.training.trainer --help
python3 -m Models.vector_planner.eval.eval \
  --checkpoint ${CKPT} --data ${IMG}:${LAB} --split-json ${SPLIT_JSON}   # holdout の経路・地図・検出の基本指標
python3 -m Models.vector_planner.deploy.export --checkpoint ${CKPT} --output ${ONNX}   # 信号ヘッド付きのみ
```

構成:

```
Models/vector_planner/
├── data/       dataset（E2EDataset・split）, geo_aug（反転/yaw/視点合成）, paste_aug（コーン貼り付け・色相入れ替え）,
│               online_aug（OnlinePaste / PoseShift）, map_labels（地図・経路・slot）, frame_label（走行帯・教師ラベル）, frenet_label
├── model/      network, heads
├── training/   trainer（サンプラ含む）, loss
├── eval/       eval（holdout の基本指標）, metrics
├── viz/        video
└── deploy/     export
tools/vector_planner/   教師データの整形・生成（実行時には import されない）
```

#### 推奨学習レシピ
基準モデルは 3 段階（全体学習 → 回避用 alpha 枝のみ追加学習を 2 回）。3 段階共通の引数を `${COMMON}` にまとめる。

```bash
DATA="--data ${IMG}:${LAB} [--data ${IMG2}:${LAB2} ...] --split-json ${SPLIT_JSON}"
COMMON="--version s --signal-head --geo-aug --batch-size 12 --weight-decay 1e-4 --workers 4 --input-height 384 --input-width 640 \
  --seed 0 --max-nonfinite 20 --teacher-lengths 6 --pose-shift-p 0.5 \
  --online-paste --paste-osm ${OSM} --paste-bank ${BANK} --paste-prob 0.7 --paste-redraws 3 --paste-group-prob 0.5 \
  --paste-hue-prob 0.5 --paste-far-degrade-prob 0.5 --paste-pylon-height 0.7 --paste-pylon-width 0.25 --paste-s-range 1.5,18 \
  --paste-person-prob 0.4 --paste-person-bank ${PERSON_BANK}"
TRAINER="python3 -m Models.vector_planner.training.trainer"

# 1) 全体学習（検出専用フレーム・信号専用フレームを混ぜる）
$TRAINER $DATA $COMMON --runs-dir ${RUNS}/base --map-epochs 4 --freeze-epochs 8 --joint-epochs 48 --lr 4e-4 \
  --corridor-weight 2.5 --agent-weight 2.0 --det-extra ${IMG}:${LAB} --det-extra-ratio 0.08 \
  --signal-only-root ${IMG}:${LAB} [--signal-only-root ...] --signal-only-ratio 0.08 --hue-swap-prob 0.5 --signal-weight 1.0

# 2) alpha 枝のみ追加学習（avoid サンプルを 0.35 に増やし、クリアランスを強める）
$TRAINER $DATA $COMMON --runs-dir ${RUNS}/a --split-alpha --alpha-only --init-from ${RUNS}/base/last.pt \
  --map-epochs 0 --freeze-epochs 0 --joint-epochs 24 --lr 3e-4 --avoid-ratio 0.35 --avoid-weight 3.0 \
  --clearance-weight 3.0 --agent-clearance-weight 0.5

# 3) 同じ設定でクリアランスをさらに強める
$TRAINER $DATA $COMMON --runs-dir ${RUNS}/b --split-alpha --alpha-only --init-from ${RUNS}/a/last.pt \
  --map-epochs 0 --freeze-epochs 0 --joint-epochs 24 --lr 2e-4 --avoid-ratio 0.35 --avoid-weight 3.0 \
  --clearance-weight 6.0 --agent-clearance-weight 1.0
```

比較・採用には `best.pt`（holdout の経路 L1 最小）ではなく `last.pt` を使う（L1 の揺れで序盤が選ばれ、検出が未学習になることがある）。

#### 教師データ整形ツール（`tools/vector_planner/`）
`learning/` を作業ディレクトリにして `python3 -m tools.vector_planner.<name>` で実行します。

```bash
# 経路・走行帯ラベルの生成 / bbox 付与 / train・holdout の split json（教師 Frenet の target_lengths 既定は 6）
python3 -m tools.vector_planner.labels build ${ROOT} ${OSM} ${OUT} [--teacher-lengths 6,8]
python3 -m tools.vector_planner.labels boxes ${LABEL_ROOT} ${OUT_ROOT} [--boxes-root ${BOXES}] [--min-score 0.5]
python3 -m tools.vector_planner.labels split --poses ${POSES} [${POSES} ...] --out ${SPLIT_JSON}

# コーン/人スプライト集（online paste 用）・静的貼り付けデータセット
python3 -m tools.vector_planner.sprites bank --image-root ${IMG} --boxes-root ${BOXES} --out ${BANK}
python3 -m tools.vector_planner.sprites bank --cls 1 --image-root ${IMG} --label-root ${LAB} --out ${PERSON_BANK}
python3 -m tools.vector_planner.sprites augment --image-root ${IMG} --label-root ${LAB} --osm ${OSM} --bank ${BANK} --out ${OUT} --pylon-height 0.7

# 検出専用フレームの追加（YOLO 形式の ds_panel + RF-DETR 擬似ラベル）
python3 -m tools.vector_planner.detection --ds-panel ${DS_PANEL} --split-json ${SPLIT_JSON} --onnx ${RFDETR_ONNX} --out-img ${IMG} --out-lab ${LAB}
```
