#!/bin/bash
# 津田沼(H100) 用。gpgpu_train_pylon.sbatch の写しで、確認ツールで引いた避ける区間・青コースだけ・
# 評価用の切り分け (train_pylon_scenes.yaml) で学習する。
# データは元の学習データ (data/) と bag から作り直した追加分 (added/) を 1 つのディレクトリに
# シンボリックリンクで並べて渡す (data_scenes/)
#SBATCH -p research
#SBATCH -t 06:00:00
#SBATCH --gpus-per-node=1
#SBATCH -c 8
#SBATCH --mem=32G
#SBATCH -J e2e_scenes
#SBATCH -o e2e_scenes_%j.out

set -euo pipefail

DATASET=${1:-$HOME/e2e_pylon/data_scenes}
RUN_NAME=${2:-pylon_scenes_$(date +%Y%m%d)}

source $HOME/smolvla_venv/bin/activate
# tensorboard だけ smolvla_venv に無いので別ディレクトリから足す。
# numpy / setuptools は venv 側を使う（target 側に入れると cv2 と torch が壊れる）
export PYTHONPATH=$HOME/e2e_pylibs:${PYTHONPATH:-}

nvidia-smi --query-gpu=name,memory.total --format=csv,noheader

cd $HOME/e2e_pylon/e2e_planner
python scripts/train_curve_weight.py "$DATASET" "$RUN_NAME" config/train_pylon_scenes.yaml
