import argparse
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


def normalize(targets: np.ndarray, steering_max: float, velocity_max: float) -> np.ndarray:
    scale = np.array([steering_max, velocity_max], dtype=np.float32)
    return np.clip(targets / scale, -1.0, 1.0)


def write_split(
    images: np.ndarray, targets: np.ndarray, idx: np.ndarray, split_dir: Path, flip: bool
) -> int:
    split_dir.mkdir(parents=True, exist_ok=True)
    n = len(idx)
    n_out = 2 * n if flip else n

    out_images = open_memmap(
        split_dir / 'images.npy', mode='w+', dtype=images.dtype,
        shape=(n_out, *images.shape[1:]))
    out_targets = open_memmap(
        split_dir / 'targets.npy', mode='w+', dtype=targets.dtype,
        shape=(n_out, targets.shape[1]))

    for start in range(0, n, BATCH_SIZE):
        batch_idx = idx[start:start + BATCH_SIZE]
        end = start + len(batch_idx)
        batch_images = images[batch_idx]
        batch_targets = targets[batch_idx]
        out_images[start:end] = batch_images
        out_targets[start:end] = batch_targets
        if flip:
            flip_targets = batch_targets.copy()
            flip_targets[:, 0] *= -1.0
            out_images[n + start:n + end] = batch_images[:, :, ::-1, :]
            out_targets[n + start:n + end] = flip_targets

    out_images.flush()
    out_targets.flush()
    return n_out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--raw-dir', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--val-ratio', type=float, default=0.2)
    parser.add_argument('--steering-max', type=float, default=15.0)
    parser.add_argument('--velocity-max', type=float, default=5.0)
    parser.add_argument('--no-flip', action='store_true')
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()

    raw_dir = Path(args.raw_dir)
    images = np.load(raw_dir / 'images.npy', mmap_mode='r')
    steers = np.load(raw_dir / 'steers.npy')
    check_aspect_ratio(images)
    targets = normalize(steers, args.steering_max, args.velocity_max)

    rng = np.random.default_rng(args.seed)
    n = len(images)
    perm = rng.permutation(n)
    n_val = int(n * args.val_ratio)
    val_idx, train_idx = perm[:n_val], perm[n_val:]

    out_dir = Path(args.out)
    for split, idx, flip in (('val', val_idx, False), ('train', train_idx, not args.no_flip)):
        split_dir = out_dir / split
        n_written = write_split(images, targets, idx, split_dir, flip)
        print(f'{split}: {n_written} samples -> {split_dir}')


if __name__ == '__main__':
    main()
