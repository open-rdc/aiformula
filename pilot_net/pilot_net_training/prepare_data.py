import argparse
import math
from pathlib import Path

import numpy as np
from numpy.lib.format import open_memmap

from lib.data import IMAGE_HEIGHT, IMAGE_WIDTH

# Number of samples processed per chunk when writing a split. Bounds peak RAM
# to roughly BATCH_SIZE images instead of loading the whole dataset at once.
BATCH_SIZE = 512


def check_aspect_ratio(images: np.ndarray):
    height, width = images.shape[1:3]
    source_ratio = width / height
    target_ratio = IMAGE_WIDTH / IMAGE_HEIGHT
    if not np.isclose(source_ratio, target_ratio, atol=1e-2):
        raise ValueError(
            f'image aspect ratio {width}x{height} ({source_ratio:.4f}) does not match '
            f'model input aspect ratio {IMAGE_WIDTH}x{IMAGE_HEIGHT} ({target_ratio:.4f})')


def normalize(targets: np.ndarray, steering_max_deg: float) -> np.ndarray:
    scale = np.float32(math.radians(steering_max_deg))
    return np.clip(targets / scale, -1.0, 1.0)


def select_augmented(source: np.ndarray, val_idx: np.ndarray) -> np.ndarray:
    """val に回った元フレーム由来の拡張サンプルを落とすマスク。

    拡張データは元フレームを描き換えたものなので、元が val にあるまま train に
    入れると同一シーンが両側に現れて val が信用できなくなる。
    """
    return ~np.isin(source, val_idx)


def write_split(segments: list, split_dir: Path) -> int:
    """(images, targets, idx) の並びを連結して1つの split に書き出す。

    左右反転は PilotNetDataset が読み出し時に作るのでここでは実体化しない。
    """
    split_dir.mkdir(parents=True, exist_ok=True)
    n = sum(len(idx) for _, _, idx in segments)

    first_images, first_targets, _ = segments[0]
    out_images = open_memmap(
        split_dir / 'images.npy', mode='w+', dtype=first_images.dtype,
        shape=(n, *first_images.shape[1:]))
    out_targets = open_memmap(
        split_dir / 'targets.npy', mode='w+', dtype=first_targets.dtype,
        shape=(n, first_targets.shape[1]))

    written = 0
    for images, targets, idx in segments:
        for start in range(0, len(idx), BATCH_SIZE):
            batch_idx = idx[start:start + BATCH_SIZE]
            end = written + len(batch_idx)
            out_images[written:end] = images[batch_idx]
            out_targets[written:end] = targets[batch_idx]
            written = end

    out_images.flush()
    out_targets.flush()
    return n


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--raw-dir', required=True)
    parser.add_argument('--aug-dir', default=None,
                        help='extract_augmented_data.py の出力。train 側にのみ足す')
    parser.add_argument('--out', required=True)
    parser.add_argument('--val-ratio', type=float, default=0.2)
    parser.add_argument('--steering-max', type=float, default=15.0,
                        help='舵角上限 [deg]。main_params.yaml の steering_max.pos と同じ単位')
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()

    raw_dir = Path(args.raw_dir)
    images = np.load(raw_dir / 'images.npy', mmap_mode='r')
    steers = np.load(raw_dir / 'steers.npy')
    check_aspect_ratio(images)
    targets = normalize(steers, args.steering_max)

    rng = np.random.default_rng(args.seed)
    n = len(images)
    perm = rng.permutation(n)
    n_val = int(n * args.val_ratio)
    val_idx, train_idx = perm[:n_val], perm[n_val:]

    train_segments = [(images, targets, train_idx)]
    if args.aug_dir is not None:
        aug_dir = Path(args.aug_dir)
        aug_images = np.load(aug_dir / 'images.npy', mmap_mode='r')
        aug_targets = normalize(np.load(aug_dir / 'steers.npy'), args.steering_max)
        aug_source = np.load(aug_dir / 'source.npy')
        check_aspect_ratio(aug_images)
        aug_idx = np.flatnonzero(select_augmented(aug_source, val_idx))
        train_segments.append((aug_images, aug_targets, aug_idx))
        print(f'augmented: {len(aug_idx)}/{len(aug_source)} kept '
              f'(dropped {len(aug_source) - len(aug_idx)} sourced from val), '
              f'real:aug = 1:{len(aug_idx) / len(train_idx):.2f}')

    out_dir = Path(args.out)
    for split, segments in (('val', [(images, targets, val_idx)]),
                            ('train', train_segments)):
        split_dir = out_dir / split
        n_written = write_split(segments, split_dir)
        print(f'{split}: {n_written} samples -> {split_dir}')


if __name__ == '__main__':
    main()
