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
python3 -m Models.training.vision_planner_trainer \
  --dataset ${DATASET} \
  --epochs 60 \
  --head argmax
```

### 推論結果の可視化
```bash
python3 -m Models.visualizations.vision_planner_visualization \
  --bag ${BAG} \
  --weights runs/lane/run1/weights/best.pt \
  --out-video ${VIDEO} \
  --with-gt
```
