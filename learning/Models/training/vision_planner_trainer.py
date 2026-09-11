import argparse
import os
import sys

import torch
import yaml
from torch.utils.data import DataLoader, WeightedRandomSampler
from torch.utils.tensorboard import SummaryWriter

from Models.model_components.vision_planner.vision_planner_network import Network
from Models.data_utils.vision_planner.load_data_vision_planner import PathDataset, find_images
from Models.training.vision_planner_loss import ComputeLoss
from Models.training.vision_planner_metrics import PathMetrics
from Models.training.vision_planner_util import AverageMeter, EMA, LinearLR, next_run_dir, set_params, setup_seed, visualize

NUM_SLOTS = 3


def device_type_of(device):
    return torch.device(device).type


def build_cosmos_sampler(paths, ratio):
    # cosmos 拡張画像はファイル名に __aug_ を含む。実データ:cosmos = 1:ratio の期待比で引く。
    # 1エポックの枚数は実データ数に固定するので、比率を変えても勾配ステップ数は変わらない
    flags = [os.path.basename(p).find('__aug_') >= 0 for p in paths]
    num_real = flags.count(False)
    num_aug = flags.count(True)
    if num_real == 0 or num_aug == 0:
        raise ValueError(f'実データ {num_real} 枚 / cosmos {num_aug} 枚 では混合比を作れない')
    weights = [ratio / num_aug if flag else 1.0 / num_real for flag in flags]
    return WeightedRandomSampler(weights, num_samples=num_real, replacement=True)


def build_loader(dataset_root, split, args, shuffle):
    paths = find_images(dataset_root, split)
    if not paths:
        raise ValueError(f'{split} の画像が見つからない: {dataset_root}')
    dataset = PathDataset(paths, NUM_SLOTS, args.input_height // 8, (args.input_height, args.input_width))
    sampler = None
    if shuffle and args.cosmos_ratio > 0:
        sampler = build_cosmos_sampler(paths, args.cosmos_ratio)
        shuffle = False
    return DataLoader(dataset, batch_size=args.batch_size, shuffle=shuffle, sampler=sampler,
                      num_workers=args.workers,
                      pin_memory=device_type_of(args.device) == 'cuda')


def train_one_epoch(model, loader, optimizer, scheduler, scaler, criterion, ema, epoch, args, log):
    model.train()

    meters = {'total': AverageMeter()}
    accumulate = max(round(64 / args.batch_size), 1)
    steps = len(loader)
    device = torch.device(args.device)
    device_type = device_type_of(args.device)

    optimizer.zero_grad()
    for index, batch in enumerate(loader):
        scheduler.step(index + steps * epoch, optimizer)
        sample, target_exist, target_valid, target_xp = [t.to(device) for t in batch]
        sample = sample.float() / 255.0

        with torch.autocast(device_type, enabled=device_type == 'cuda'):
            outputs = model(sample)
            total, parts = criterion(outputs, (target_exist, target_valid, target_xp))

        scaler.scale(total).backward()
        if index % accumulate == 0:
            scaler.step(optimizer)
            scaler.update()
            optimizer.zero_grad()
            if ema is not None:
                ema.update(model)

        meters['total'].update(float(total), sample.size(0))
        for name, value in parts.items():
            meters.setdefault(name, AverageMeter()).update(value, sample.size(0))

        if log is not None and index == 0:
            image = visualize(batch[0][0], [o[0] for o in outputs],
                              (target_exist[0], target_valid[0], target_xp[0]), args.input_height // 8)
            log.add_image('train/overlay', image, global_step=epoch + 1, dataformats='HWC')

    return {name: meter.avg for name, meter in meters.items()}


@torch.no_grad()
def validate(model, loader, num_slots, device='cpu', input_width=640, min_slot_rows=None):
    model.eval()

    kwargs = {'input_width': input_width}
    if min_slot_rows is not None:
        kwargs['min_slot_rows'] = min_slot_rows
    metrics = PathMetrics(num_slots, **kwargs)
    for batch in loader:
        sample, target_exist, target_valid, target_xp = [t.to(device) for t in batch]
        outputs = model(sample.float() / 255.0)
        metrics.update(outputs, (target_exist, target_valid, target_xp))

    return metrics.compute(), metrics.score()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', required=True)
    parser.add_argument('--params', default='configs/train.yaml')
    parser.add_argument('--runs-dir', default='runs/lane')
    parser.add_argument('--version', default='n')
    parser.add_argument('--head', default='argmax', choices=('argmax', 'dist'))
    parser.add_argument('--backbone', default='ctx', choices=('ctx', 'c3k2'))
    parser.add_argument('--cosmos-ratio', type=float, default=0.0, help='実データ1に対するcosmos拡張の比重。0で拡張を使わない')
    parser.add_argument('--epochs', type=int, default=30)
    parser.add_argument('--batch-size', type=int, default=16)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--input-height', type=int, default=384)
    parser.add_argument('--input-width', type=int, default=640)
    parser.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    args = parser.parse_args(argv)

    setup_seed()
    with open(args.params) as f:
        params = yaml.safe_load(f)

    run_dir = next_run_dir(args.runs_dir)
    weights_dir = os.path.join(run_dir, 'weights')
    os.makedirs(weights_dir, exist_ok=True)
    log = SummaryWriter(log_dir=run_dir)

    device = torch.device(args.device)
    model = Network(version=args.version, head_mode=args.head, backbone=args.backbone).to(device)
    train_loader = build_loader(args.dataset, 'train', args, shuffle=True)
    val_loader = build_loader(args.dataset, 'val', args, shuffle=False)

    optimizer = torch.optim.SGD(set_params(model, params['weight_decay']),
                                params['min_lr'], params['momentum'], nesterov=True)
    scheduler = LinearLR(params, args.epochs, len(train_loader))
    scaler = torch.amp.GradScaler(enabled=device_type_of(args.device) == 'cuda')
    criterion = ComputeLoss(params)
    ema = EMA(model)

    best = float('inf')
    for epoch in range(args.epochs):
        losses = train_one_epoch(model, train_loader, optimizer, scheduler, scaler, criterion, ema, epoch, args, log)
        result, score = validate(ema.ema, val_loader, NUM_SLOTS, args.device, args.input_width)

        for name, value in losses.items():
            log.add_scalar(f'loss/{name}', value, epoch + 1)
        for name, value in result.items():
            log.add_scalar(f'val/{name}', value, epoch + 1)

        print(f'epoch {epoch + 1}/{args.epochs}  loss {losses["total"]:.4f}'
              f'  横位置誤差 {result["lateral_macro_px"]:.1f}px (プール {result["lateral_px"]:.1f}px)'
              f'  valid_f1 {result["valid_f1"]:.3f}  exist_f1 {result["exist_f1"]:.3f}')

        checkpoint = {'epoch': epoch + 1, 'version': args.version, 'head': args.head, 'backbone': args.backbone,
                      'model_state_dict': ema.ema.state_dict()}
        torch.save(checkpoint, os.path.join(weights_dir, 'last.pt'))
        if score < best:
            best = score
            torch.save(checkpoint, os.path.join(weights_dir, 'best.pt'))

    log.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
