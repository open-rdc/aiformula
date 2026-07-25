#!/usr/bin/env bash
set -euo pipefail

OUT_DIR="${1:-${HOME}/bag_$(date +%Y%m%d_%H%M%S)}"

IMAGE_TOPIC="/zed/zed_node/rgb/image_rect_color"
CMD_TOPIC="/cmd_vel"

echo "=== Recording ${IMAGE_TOPIC}, ${CMD_TOPIC} -> ${OUT_DIR} ==="
echo "=== Ctrl+C で記録終了 ==="

ros2 bag record \
    --storage sqlite3 \
    --output "$OUT_DIR" \
    "$IMAGE_TOPIC" \
    "$CMD_TOPIC"
