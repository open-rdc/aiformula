import argparse
from pathlib import Path

import torch
import yaml
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
from tqdm import tqdm

from lib.data import PilotNetDataset
from lib.loss import build_loss
from lib.model import PilotNet


def run_epoch(model, loader, criterion, optimizer, device, train: bool):
    model.train(train)
    total_loss = 0.0
    with torch.set_grad_enabled(train):
        for images, targets in tqdm(loader, leave=False):
            images, targets = images.to(device), targets.to(device)
            preds = model(images)
            loss = criterion(preds, targets)
            if train:
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
            total_loss += loss.item() * len(images)
    return total_loss / len(loader.dataset)


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

    train_loader = DataLoader(
        PilotNetDataset(cfg['data']['train_dir']),
        batch_size=cfg['train']['batch_size'],
        shuffle=True,
        num_workers=cfg['train']['num_workers'],
    )
    val_loader = DataLoader(
        PilotNetDataset(cfg['data']['val_dir']),
        batch_size=cfg['train']['batch_size'],
        shuffle=False,
        num_workers=cfg['train']['num_workers'],
    )

    model = PilotNet(output_dim=cfg['model']['output_dim']).to(device)
    criterion = build_loss(**cfg['loss'])
    optimizer = torch.optim.Adam(model.parameters(), lr=cfg['train']['lr'])

    save_dir = Path(cfg['train']['save_dir'])
    save_dir.mkdir(parents=True, exist_ok=True)
    writer = SummaryWriter(cfg['train']['log_dir'])

    best_val_loss = float('inf')
    for epoch in range(cfg['train']['epochs']):
        train_loss = run_epoch(model, train_loader, criterion, optimizer, device, train=True)
        val_loss = run_epoch(model, val_loader, criterion, optimizer, device, train=False)

        writer.add_scalar('loss/train', train_loss, epoch)
        writer.add_scalar('loss/val', val_loss, epoch)
        print(f'epoch {epoch}: train_loss={train_loss:.5f} val_loss={val_loss:.5f}')

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save(model.state_dict(), save_dir / 'best_model.pth')

    torch.save(model.state_dict(), save_dir / 'last_model.pth')
    writer.close()


if __name__ == '__main__':
    main()
