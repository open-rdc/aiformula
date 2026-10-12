import argparse
import os
import sys

import torch
import yaml
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter

from Models.model_components.vision_planner.vision_planner_network import Network
from Models.data_utils.vision_planner.load_data_vision_planner import PathDataset, branch_sampler, find_images
from Models.training.vision_planner_loss import compute_loss
from Models.training.vision_planner_metrics import PathMetrics
from Models.training.vision_planner_util import AverageMeter, EMA, LinearLR, next_run_dir, set_params, setup_seed, visualize

NUM_SLOTS = 3


def device_type_of(device):
    return torch.device(device).type


def build_loader(dataset_root, split, args, train):
    paths = find_images(dataset_root, split)
    dataset = PathDataset(paths, NUM_SLOTS, args.input_height // 8, (args.input_height, args.input_width))
    sampler = branch_sampler(paths, NUM_SLOTS, args.input_height // 8) if train else None
    return DataLoader(dataset, batch_size=args.batch_size, shuffle=False, sampler=sampler,
                      num_workers=args.workers,
                      pin_memory=device_type_of(args.device) == 'cuda')


def train_one_epoch(model, loader, optimizer, scheduler, scaler, ema, epoch, args, log, branch_scale,
                    smooth_scale):
    model.train()

    meters = {name: AverageMeter() for name in ('total', 'xp', 'valid', 'smooth')}
    accumulate = max(round(64 / args.batch_size), 1)
    steps = len(loader)
    device = torch.device(args.device)
    device_type = device_type_of(args.device)

    optimizer.zero_grad()
    for index, batch in enumerate(loader):
        scheduler.step(index + steps * epoch, optimizer)
        sample, target_valid, target_xp = [t.to(device) for t in batch]
        sample = sample.float() / 255.0

        with torch.autocast(device_type, enabled=device_type == 'cuda'):
            outputs = model(sample)
            total, parts = compute_loss(outputs, (target_valid, target_xp), branch_scale, smooth_scale)

        scaler.scale(total).backward()
        if index % accumulate == 0:
            scaler.step(optimizer)
            scaler.update()
            optimizer.zero_grad()
            if ema is not None:
                ema.update(model)

        meters['total'].update(float(total), sample.size(0))
        for name, value in parts.items():
            meters[name].update(value, sample.size(0))

        if log is not None and index == 0:
            sample_output = (outputs[0][0], outputs[1][0])
            image = visualize(batch[0][0], sample_output, (target_valid[0], target_xp[0]), args.input_height // 8)
            log.add_image('train/overlay', image, global_step=epoch + 1, dataformats='HWC')

    return {name: meter.avg for name, meter in meters.items()}


@torch.no_grad()
def validate(model, loader, num_slots, device='cpu', input_width=640):
    model.eval()

    metrics = PathMetrics(num_slots, input_width=input_width)
    for batch in loader:
        sample, target_valid, target_xp = [t.to(device) for t in batch]
        outputs = model(sample.float() / 255.0)
        metrics.update(outputs, (target_valid, target_xp))

    return metrics.compute(), metrics.score()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', required=True)
    parser.add_argument('--params', default='configs/train.yaml')
    parser.add_argument('--runs-dir', default='runs/lane')
    parser.add_argument('--version', default='n')
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
    model = Network(version=args.version).to(device)
    train_loader = build_loader(args.dataset, 'train', args, train=True)
    val_loader = build_loader(args.dataset, 'val', args, train=False)

    optimizer = torch.optim.SGD(set_params(model, params['weight_decay']),
                                params['min_lr'], params['momentum'], nesterov=True)
    scheduler = LinearLR(params, args.epochs, len(train_loader))
    scaler = torch.amp.GradScaler(enabled=device_type_of(args.device) == 'cuda')
    ema = EMA(model)

    best = float('inf')
    for epoch in range(args.epochs):
        losses = train_one_epoch(model, train_loader, optimizer, scheduler, scaler, ema, epoch, args, log,
                                 params['branch_scale'], params['smooth_scale'])
        result, score = validate(ema.ema, val_loader, NUM_SLOTS, args.device, args.input_width)

        for name, value in losses.items():
            log.add_scalar(f'loss/{name}', value, epoch + 1)
        for name, value in result.items():
            log.add_scalar(f'val/{name}', value, epoch + 1)
        log.add_scalar('val/score', score, epoch + 1)

        print(f'epoch {epoch + 1}/{args.epochs}  loss {losses["total"]:.4f}'
              f' (xp {losses["xp"]:.4f}  valid {losses["valid"]:.4f}  smooth {losses["smooth"]:.4f})'
              f'  横位置誤差 {result["lateral_px"]:.1f}px'
              f'  分離r left {result["separation_r/left"]:.3f} right {result["separation_r/right"]:.3f}'
              f'  分離px left {result["separation/left"]:.1f}/{result["separation_gt/left"]:.1f}'
              f' right {result["separation/right"]:.1f}/{result["separation_gt/right"]:.1f}'
              f'  valid_f1 {result["valid_f1"]:.3f}')

        checkpoint = {'epoch': epoch + 1, 'version': args.version,
                      'model_state_dict': ema.ema.state_dict()}
        torch.save(checkpoint, os.path.join(weights_dir, 'last.pt'))
        if score < best:
            best = score
            torch.save(checkpoint, os.path.join(weights_dir, 'best.pt'))

    log.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
