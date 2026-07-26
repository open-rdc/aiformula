import argparse
from pathlib import Path

import numpy as np
import torch
import yaml
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
from tqdm import tqdm

from lib.data import PilotNetDataset
from lib.loss import build_loss
from lib.model import PilotNet
from lib.weighting import NUM_BINS, BinWeighter


def run_epoch(model, loader, criterion, optimizer, device, train: bool):
    model.train(train)
    total_weighted = 0.0
    total_plain = 0.0
    with torch.set_grad_enabled(train):
        for images, targets, weights in tqdm(loader, leave=False):
            images, targets = images.to(device), targets.to(device)
            weights = weights.to(device)
            preds = model(images)
            loss = criterion(preds, targets, weights)
            if train:
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
            total_weighted += loss.item() * len(images)
            with torch.no_grad():
                total_plain += criterion(preds, targets).item() * len(images)
    n = len(loader.dataset)
    return total_weighted / n, total_plain / n


def report_bins(weighter, targets, beta):
    counts = np.bincount(weighter.bin_indices(targets), minlength=NUM_BINS)
    total = counts.sum()
    print(f'steering balance: beta={beta}')
    for k in range(NUM_BINS):
        lo, hi = weighter.edges[k], weighter.edges[k + 1]
        share = counts[k] * weighter.weights[k] / total if total else 0.0
        print(f'  bin{k} |steer| {lo:.1f}-{hi:.1f}: n={counts[k]:>7,} '
              f'weight={weighter.weights[k]:.3f} loss_share={share * 100:5.1f}%')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='config/train.yaml')
    parser.add_argument('--save-dir', default=None)
    parser.add_argument('--log-dir', default=None)
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    if args.save_dir is not None:
        cfg['train']['save_dir'] = args.save_dir
    if args.log_dir is not None:
        cfg['train']['log_dir'] = args.log_dir

    torch.manual_seed(cfg['train']['seed'])
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    loss_cfg = dict(cfg['loss'])
    beta = float(loss_cfg.pop('beta', 0.0))

    train_targets = np.load(Path(cfg['data']['train_dir']) / 'targets.npy')
    val_targets = np.load(Path(cfg['data']['val_dir']) / 'targets.npy')
    weighter = BinWeighter.fit(train_targets, beta)
    report_bins(weighter, train_targets, beta)

    train_loader = DataLoader(
        PilotNetDataset(cfg['data']['train_dir'], weighter.weights_for(train_targets), flip=True),
        batch_size=cfg['train']['batch_size'],
        shuffle=True,
        num_workers=cfg['train']['num_workers'],
    )
    val_loader = DataLoader(
        PilotNetDataset(cfg['data']['val_dir'], weighter.weights_for(val_targets)),
        batch_size=cfg['train']['batch_size'],
        shuffle=False,
        num_workers=cfg['train']['num_workers'],
    )

    model = PilotNet(output_dim=cfg['model']['output_dim']).to(device)
    criterion = build_loss(**loss_cfg).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=cfg['train']['lr'])

    save_dir = Path(cfg['train']['save_dir'])
    save_dir.mkdir(parents=True, exist_ok=True)
    writer = SummaryWriter(cfg['train']['log_dir'])

    # 重み付き val_loss は beta に依存するので、beta を跨いで比較するときは
    # 重みなし val_loss で選んだ best_unweighted_model.pth を使う。
    best_val_loss = float('inf')
    best_val_plain = float('inf')
    for epoch in range(cfg['train']['epochs']):
        train_loss, train_plain = run_epoch(
            model, train_loader, criterion, optimizer, device, train=True)
        val_loss, val_plain = run_epoch(
            model, val_loader, criterion, optimizer, device, train=False)

        writer.add_scalar('loss/train', train_loss, epoch)
        writer.add_scalar('loss/train_unweighted', train_plain, epoch)
        writer.add_scalar('loss/val', val_loss, epoch)
        writer.add_scalar('loss/val_unweighted', val_plain, epoch)
        print(f'epoch {epoch}: train_loss={train_loss:.5f} val_loss={val_loss:.5f} '
              f'(unweighted train={train_plain:.5f} val={val_plain:.5f})')

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save(model.state_dict(), save_dir / 'best_model.pth')

        if val_plain < best_val_plain:
            best_val_plain = val_plain
            torch.save(model.state_dict(), save_dir / 'best_unweighted_model.pth')

    torch.save(model.state_dict(), save_dir / 'last_model.pth')
    writer.close()


if __name__ == '__main__':
    main()
