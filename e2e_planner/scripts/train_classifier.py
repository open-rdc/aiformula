
#!/usr/bin/env python3
"""
ジャンクション分類器 (JunctionClassifier) のトレーニングスクリプト。

ワークフロー:
  1. python3 binarize_dataset.py --session <session_dir>  ← YOLOP二値化
  2. python3 train_classifier.py <session_dir> [<session_dir2> ...]

入力:
  session/masks/XXXXXX.jpg   (binarize_dataset.py で生成した64×48二値マスク)
  session/data_binarized.csv  (なければ data.csv を使用)
    必要カラム: image, label, [mask]

出力:
  <package_root>/weights/junction_classifier.pt  (TorchScript形式)

ラベル:
  0 = 道なり (straight)
  1 = 右折   (right)   ← buttons[1] で付与
  2 = 左折   (left)    ← buttons[2] で付与
"""

import sys
import csv
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader, random_split, ConcatDataset
from torch.utils.tensorboard import SummaryWriter
import cv2
import pandas as pd
import numpy as np
from pathlib import Path
from tqdm import tqdm

sys.path.insert(0, str(Path(__file__).parent))
from network import JunctionClassifier

IMAGE_W = 64
IMAGE_H = 48
LABEL_NAMES = {0: 'straight', 1: 'right', 2: 'left'}


class JunctionDataset(Dataset):
    """セッションディレクトリから二値マスク＋ラベルを読み込むデータセット。"""

    def __init__(self, session_dir: Path) -> None:
        self.session_dir = session_dir

        csv_path = session_dir / 'data_binarized.csv'
        if not csv_path.exists():
            csv_path = session_dir / 'data.csv'
            print(f'  [{session_dir.name}] data_binarized.csv なし → data.csv を使用')

        if not csv_path.exists():
            raise FileNotFoundError(f'CSV が見つかりません: {session_dir}')

        df = pd.read_csv(csv_path)
        self.samples: list[tuple[Path, int]] = []

        for _, row in df.iterrows():
            label = int(row['label'])

            # マスクパスの決定: mask列 → images列から推測
            if 'mask' in df.columns and pd.notna(row.get('mask')) and str(row['mask']) != '':
                mask_path = session_dir / str(row['mask'])
            else:
                img_stem = Path(str(row['image'])).stem
                mask_path = session_dir / 'masks' / (img_stem + '.jpg')

            if mask_path.exists():
                self.samples.append((mask_path, label))

        if not self.samples:
            raise RuntimeError(
                f'有効なサンプルが見つかりません: {session_dir}\n'
                '先に binarize_dataset.py を実行してください。'
            )

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> tuple:
        mask_path, label = self.samples[idx]
        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if mask is None:
            mask = np.zeros((IMAGE_H, IMAGE_W), dtype=np.uint8)
        mask = cv2.resize(mask, (IMAGE_W, IMAGE_H), interpolation=cv2.INTER_NEAREST)
        mask = (mask > 127).astype(np.float32)
        tensor = torch.from_numpy(mask).unsqueeze(0)  # 1×H×W
        return tensor, label


def _print_label_dist(dataset: Dataset) -> None:
    counts = {l: 0 for l in LABEL_NAMES}
    for i in range(len(dataset)):
        _, label = dataset[i]
        counts[label] = counts.get(label, 0) + 1
    total = len(dataset)
    print('ラベル分布:')
    for l, name in LABEL_NAMES.items():
        n = counts.get(l, 0)
        print(f'  {name:8s}: {n:5d}  ({n / total * 100:.1f}%)')


def train(
    session_dirs: list[str],
    epochs: int = 50,
    batch_size: int = 32,
    lr: float = 1e-3,
    weight_file: str = 'junction_classifier.pt',
    num_workers: int = 2,
) -> None:
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f'デバイス: {device}')

    datasets = [JunctionDataset(Path(d)) for d in session_dirs]
    total_samples = sum(len(ds) for ds in datasets)
    print(f'総サンプル数: {total_samples} ({len(datasets)} セッション)')

    combined = datasets[0] if len(datasets) == 1 else ConcatDataset(datasets)
    _print_label_dist(combined)

    n_train = int(0.8 * len(combined))
    n_val = len(combined) - n_train
    train_ds, val_ds = random_split(combined, [n_train, n_val])
    print(f'Train: {n_train}  Val: {n_val}')

    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True, num_workers=num_workers, drop_last=True)
    val_loader = DataLoader(val_ds, batch_size=batch_size, shuffle=False, num_workers=num_workers)

    model = JunctionClassifier().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    criterion = nn.CrossEntropyLoss()

    script_dir = Path(__file__).parent
    package_root = script_dir.parent
    weights_dir = package_root / 'weights'
    weights_dir.mkdir(exist_ok=True)
    weight_path = weights_dir / weight_file

    writer = SummaryWriter(log_dir=str(package_root / 'runs' / 'junction_classifier'))
    print(f'TensorBoard ログ: {package_root / "runs" / "junction_classifier"}')
    print(f'TensorBoard 起動コマンド: tensorboard --logdir {package_root / "runs"}')
    print('ブラウザで http://localhost:6006 を開いて進捗を確認できます')

    best_val_acc = 0.0

    for epoch in range(1, epochs + 1):
        model.train()
        total_loss = 0.0
        train_correct = 0

        pbar = tqdm(train_loader, desc=f'Epoch {epoch:3d}/{epochs} [Train]', leave=False)
        for images, labels in pbar:
            images = images.to(device)
            labels = labels.to(device)
            optimizer.zero_grad()
            outputs = model(images)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            total_loss += loss.item()
            train_correct += int((outputs.argmax(1) == labels).sum())
            pbar.set_postfix({'loss': f'{loss.item():.4f}'})

        scheduler.step()
        train_loss = total_loss / len(train_loader)
        train_acc = train_correct / n_train

        model.eval()
        val_correct = 0
        with torch.no_grad():
            for images, labels in val_loader:
                images = images.to(device)
                labels = labels.to(device)
                outputs = model(images)
                val_correct += int((outputs.argmax(1) == labels).sum())
        val_acc = val_correct / n_val

        writer.add_scalar('Loss/train', train_loss, epoch)
        writer.add_scalar('Accuracy/train', train_acc, epoch)
        writer.add_scalar('Accuracy/val', val_acc, epoch)

        marker = ' *' if val_acc > best_val_acc else ''
        print(f'Epoch {epoch:3d}: train_acc={train_acc:.3f}  val_acc={val_acc:.3f}{marker}')

        if val_acc > best_val_acc:
            best_val_acc = val_acc
            scripted = torch.jit.script(model)
            scripted.save(str(weight_path))

    writer.close()
    print(f'\nトレーニング完了。最良バリデーション精度: {best_val_acc:.3f}')
    print(f'モデル保存先: {weight_path}')


def main() -> None:
    import argparse
    import yaml

    script_dir = Path(__file__).parent
    default_config = script_dir.parent / 'config' / 'train_classifier.yaml'

    parser = argparse.ArgumentParser(
        description='JunctionClassifier の学習',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('sessions', nargs='+', help='セッションディレクトリ (複数指定可)')
    parser.add_argument('--config', default=str(default_config),
                        help=f'コンフィグYAMLファイルのパス (デフォルト: {default_config})')
    parser.add_argument('--epochs', type=int, default=None)
    parser.add_argument('--batch-size', type=int, default=None)
    parser.add_argument('--lr', type=float, default=None)
    parser.add_argument('--num-workers', type=int, default=None)
    parser.add_argument('--weight-file', default=None,
                        help='出力モデルファイル名 (weights/ 以下)')
    args = parser.parse_args()

    cfg: dict = {}
    config_path = Path(args.config)
    if config_path.exists():
        with open(config_path) as f:
            cfg = yaml.safe_load(f) or {}
        print(f'コンフィグ読み込み: {config_path}')
    else:
        print(f'警告: コンフィグファイルが見つかりません: {config_path}')

    epochs = args.epochs if args.epochs is not None else cfg.get('epochs', 50)
    batch_size = args.batch_size if args.batch_size is not None else cfg.get('batch_size', 32)
    lr = args.lr if args.lr is not None else cfg.get('learning_rate', 1e-3)
    num_workers = args.num_workers if args.num_workers is not None else cfg.get('num_workers', 2)
    weight_file = args.weight_file if args.weight_file is not None else cfg.get('weight_file', 'junction_classifier.pt')

    for d in args.sessions:
        if not Path(d).exists():
            print(f'エラー: {d} が見つかりません')
            sys.exit(1)

    train(args.sessions, epochs, batch_size, lr, weight_file, num_workers)


if __name__ == '__main__':
    main()
