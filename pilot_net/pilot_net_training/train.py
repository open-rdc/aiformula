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
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    torch.manual_seed(cfg['train']['seed'])
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    dataset_dir = Path(cfg['data']['dataset_dir'])
    train_loader = DataLoader(
        PilotNetDataset(dataset_dir / 'train'),
        batch_size=cfg['train']['batch_size'],
        shuffle=True,
        num_workers=cfg['train']['num_workers'],
    )
    val_loader = DataLoader(
        PilotNetDataset(dataset_dir / 'val'),
        batch_size=cfg['train']['batch_size'],
        shuffle=False,
        num_workers=cfg['train']['num_workers'],
    )

    model = PilotNet(output_dim=cfg['model']['output_dim']).to(device)
    criterion = build_loss(cfg['train']['loss'], weights=cfg['train'].get('loss_weights'))
    optimizer = torch.optim.Adam(model.parameters(), lr=cfg['train']['lr'])

    checkpoint_dir = Path(cfg['train']['checkpoint_dir'])
    checkpoint_dir.mkdir(parents=True, exist_ok=True)
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
            torch.save(model.state_dict(), checkpoint_dir / 'best_model.pth')

    torch.save(model.state_dict(), checkpoint_dir / 'last_model.pth')
    writer.close()


if __name__ == '__main__':
    main()
