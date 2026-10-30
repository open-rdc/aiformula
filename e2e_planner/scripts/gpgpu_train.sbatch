#!/bin/bash
# 津田沼(H100) 用。新習志野とは HOME も venv も別なので使い回さないこと
#SBATCH -p research
#SBATCH -t 02:00:00
#SBATCH --gpus-per-node=1
#SBATCH -c 8
#SBATCH --mem=32G
#SBATCH -J e2e300
#SBATCH -o e2e300_%j.out

set -euo pipefail

DATASET=${1:-$HOME/e2e_planner/data/20260904_135822_dataset}
RUN_NAME=${2:-sim20260904_rescale_300ep}

source $HOME/smolvla_venv/bin/activate
# tensorboard だけ smolvla_venv に無いので別ディレクトリから足す。
# numpy / setuptools は venv 側を使う（target 側に入れると cv2 と torch が壊れる）
export PYTHONPATH=$HOME/e2e_pylibs:${PYTHONPATH:-}

nvidia-smi --query-gpu=name,memory.total --format=csv,noheader

cd $HOME/e2e_planner/scripts
python train.py "$DATASET" "$RUN_NAME"
