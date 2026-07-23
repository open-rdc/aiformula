import argparse
from pathlib import Path

import numpy as np

from lib.data import IMAGE_HEIGHT, IMAGE_WIDTH


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


def flip_augment(images: np.ndarray, targets: np.ndarray):
    flipped_images = images[:, :, ::-1, :]
    flipped_targets = targets.copy()
    flipped_targets[:, 0] *= -1.0
    return flipped_images, flipped_targets


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
    images = np.load(raw_dir / 'images.npy')
    steers = np.load(raw_dir / 'steers.npy')
    check_aspect_ratio(images)
    targets = normalize(steers, args.steering_max, args.velocity_max)

    rng = np.random.default_rng(args.seed)
    n = len(images)
    perm = rng.permutation(n)
    n_val = int(n * args.val_ratio)
    val_idx, train_idx = perm[:n_val], perm[n_val:]

    train_images, train_targets = images[train_idx], targets[train_idx]
    val_images, val_targets = images[val_idx], targets[val_idx]

    if not args.no_flip:
        flip_images, flip_targets = flip_augment(train_images, train_targets)
        train_images = np.concatenate([train_images, flip_images], axis=0)
        train_targets = np.concatenate([train_targets, flip_targets], axis=0)

    out_dir = Path(args.out)
    for split, imgs, tgts in (('train', train_images, train_targets), ('val', val_images, val_targets)):
        split_dir = out_dir / split
        split_dir.mkdir(parents=True, exist_ok=True)
        np.save(split_dir / 'images.npy', imgs)
        np.save(split_dir / 'targets.npy', tgts)
        print(f'{split}: {len(imgs)} samples -> {split_dir}')


if __name__ == '__main__':
    main()
