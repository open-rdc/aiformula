#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "usage: run_pipeline.bash <bag_path>... [-- val_ratio [config] [out_weights]]" >&2
    exit 1
}

BAG_PATHS=()
while [[ $# -gt 0 && "$1" != "--" ]]; do
    BAG_PATHS+=("$1")
    shift
done
if [[ ${#BAG_PATHS[@]} -eq 0 ]]; then
    usage
fi
if [[ "${1:-}" == "--" ]]; then
    shift
fi

VAL_RATIO="${1:-0.2}"
CONFIG="${2:-config/train.yaml}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

OUT_WEIGHTS="${3:-../pilot_net_controller/weights/pilotnet_weights.npy}"

read -r SAVE_DIR LOG_DIR OUTPUT_DIM < <(python3 -c "
import sys
import yaml

with open(sys.argv[1]) as f:
    cfg = yaml.safe_load(f)
print(cfg['train']['save_dir'], cfg['train']['log_dir'], cfg['model']['output_dim'])
" "$CONFIG")

echo "=== Config: bags=${BAG_PATHS[*]}, val_ratio=${VAL_RATIO}, config=${CONFIG}, output_dim=${OUTPUT_DIM} ==="

echo "=== 1. Extract data from bag ==="
rm -rf data/raw
python3 extract_data_from_bag.py --bag "${BAG_PATHS[@]}" --out data/raw

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
