# pilot_net_training

PilotNet の学習パイプライン。rosbag から学習済み重みを生成する。

```bash
pip install -e .

python3 extract_data_from_bag.py --bag <bag_path> --out data/raw

python3 prepare_data.py --raw-dir data/raw --out data/dataset

python3 train.py --config config/train.yaml

python3 convert_weight.py \
  --checkpoint checkpoints/best_model.pth \
  --out ../pilot_net_controller/weights/pilotnet_weights.npy
```

まとめて実行する場合:

```bash
./run_pipeline.bash <bag_path>
```
