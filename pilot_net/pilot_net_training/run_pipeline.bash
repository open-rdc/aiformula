#!/usr/bin/env bash
set -euo pipefail

BAG_PATH="${1:?usage: run_pipeline.bash <bag_path> [val_ratio] [config] [out_weights]}"
VAL_RATIO="${2:-0.2}"
CONFIG="${3:-config/train.yaml}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

OUT_WEIGHTS="${4:-../pilot_net_controller/weights/pilotnet_weights.npy}"

read -r SAVE_DIR LOG_DIR OUTPUT_DIM < <(python3 -c "
import sys
import yaml

with open(sys.argv[1]) as f:
    cfg = yaml.safe_load(f)
print(cfg['train']['save_dir'], cfg['train']['log_dir'], cfg['model']['output_dim'])
" "$CONFIG")

echo "=== Config: bag=${BAG_PATH}, val_ratio=${VAL_RATIO}, config=${CONFIG}, output_dim=${OUTPUT_DIM} ==="

echo "=== 1. Extract data from bag ==="
rm -rf data/raw
python3 extract_data_from_bag.py --bag "$BAG_PATH" --out data/raw

echo "=== 2. Prepare data (train/val split) ==="
rm -rf data/dataset
python3 prepare_data.py \
    --raw-dir data/raw \
    --out data/dataset \
    --val-ratio "$VAL_RATIO"

echo "=== 3. Train ==="
rm -rf "$SAVE_DIR" "$LOG_DIR"
trap 'echo "Training interrupted, continuing pipeline..."' INT
python3 train.py --config "$CONFIG"
trap - INT

echo "=== 4. Convert weights ==="
mkdir -p "$(dirname "$OUT_WEIGHTS")"
python3 convert_weight.py \
    --checkpoint "${SAVE_DIR}/best_model.pth" \
    --out "$OUT_WEIGHTS" \
    --output-dim "$OUTPUT_DIM"

echo "=== Pipeline complete ==="
