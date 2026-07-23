#!/usr/bin/env bash
set -euo pipefail

BAG_PATH="${1:?usage: run_pipeline.bash <bag_path>}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

python3 extract_data_from_bag.py --bag "$BAG_PATH" --out data/raw
python3 prepare_data.py --raw-dir data/raw --out data/dataset
python3 train.py --config config/train.yaml
python3 convert_weight.py \
  --checkpoint checkpoints/best_model.pth \
  --out ../pilot_net_controller/weights/pilotnet_weights.npy
