#!/bin/bash
# 津田沼(H100) 用。gpgpu_train.sbatch の写しで、パイロンマスク入りの設定 (train_pylon.yaml) で学習する。
# データの各 bag ディレクトリに pylon_masks/ (scripts/create_pylon_masks.py) が要る
#SBATCH -p research
#SBATCH -t 06:00:00
#SBATCH --gpus-per-node=1
#SBATCH -c 8
#SBATCH --mem=32G
#SBATCH -J e2e_pylon
#SBATCH -o e2e_pylon_%j.out

set -euo pipefail

DATASET=${1:-$HOME/e2e_pylon/data}
RUN_NAME=${2:-pylon_$(date +%Y%m%d)}

source $HOME/smolvla_venv/bin/activate
# tensorboard だけ smolvla_venv に無いので別ディレクトリから足す。
# numpy / setuptools は venv 側を使う（target 側に入れると cv2 と torch が壊れる）
export PYTHONPATH=$HOME/e2e_pylibs:${PYTHONPATH:-}

nvidia-smi --query-gpu=name,memory.total --format=csv,noheader

cd $HOME/e2e_pylon/e2e_planner
python scripts/train_curve_weight.py "$DATASET" "$RUN_NAME" config/train_pylon.yaml
