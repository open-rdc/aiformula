#!/usr/bin/env python3

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, random_split
from torch.utils.tensorboard import SummaryWriter

import pandas as pd
import numpy as np
import sys
import os
from datetime import datetime
import argparse

from tqdm import tqdm

class MotionSequenceDataset(Dataset):
    def __init__(self, csv_path, seq_len=10):
        df = pd.read_csv(csv_path)
        self.seq_len = seq_len

        cols = ['v0', 'w0', 'acc_v0', 'acc_w0', 'potentio_th0',
                'linear_x', 'angular_z']
        self.features = df[cols].values.astype(np.float32)
        self.targets = df[['v1', 'w1', 'potentio_th1']].values.astype(np.float32)

        self.inputs_seq = []
        self.targets_seq = []
        for i in range(len(df) - seq_len):
            self.inputs_seq.append(self.features[i:i+seq_len])
            self.targets_seq.append(self.targets[i+seq_len])

    def __len__(self):
        return len(self.inputs_seq)

    def __getitem__(self, idx):
        return self.inputs_seq[idx], self.targets_seq[idx]


class LSTMModel(nn.Module):
    def __init__(self, input_dim=7, hidden_dim=64, output_dim=3, num_layers=1):
        super().__init__()
        self.lstm = nn.LSTM(input_dim, hidden_dim, num_layers, batch_first=True)
        self.fc = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, output_dim)
        )

    def forward(self, x):
        # x: [batch_size, seq_len, input_dim]
        out, _ = self.lstm(x)
        last_hidden = out[:, -1, :]
        return self.fc(last_hidden)


def train_model(csv_path, model_name, num_epochs=100, batch_size=32, lr=1e-3, seq_len=10):
    dataset = MotionSequenceDataset(csv_path, seq_len=seq_len)

    train_size = int(0.8 * len(dataset))
    test_size = len(dataset) - train_size
    train_dataset, test_dataset = random_split(dataset, [train_size, test_size])

    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    test_loader = DataLoader(test_dataset, batch_size=batch_size)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    input_dim = 7
    model = LSTMModel(input_dim=input_dim).to(device)
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=lr)

    log_dir = f"runs/train_lstm_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    writer = SummaryWriter(log_dir=log_dir)

    for epoch in range(num_epochs):
        model.train()
        train_loss = 0.0

        pbar = tqdm(train_loader, desc=f"[Epoch {epoch+1}/{num_epochs}]", leave=False)
        for x, y in pbar:
            x, y = x.to(device), y.to(device)
            optimizer.zero_grad()
            outputs = model(x)
            loss = criterion(outputs, y)
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * x.size(0)
            pbar.set_postfix({'BatchLoss': loss.item()})

        train_loss /= train_size
        writer.add_scalar("Loss/train", train_loss, epoch)

        model.eval()
        test_loss = 0.0
        with torch.no_grad():
            for x, y in test_loader:
                x, y = x.to(device), y.to(device)
                outputs = model(x)
                loss = criterion(outputs, y)
                test_loss += loss.item() * x.size(0)
        test_loss /= test_size
        writer.add_scalar("Loss/test", test_loss, epoch)

        print(f"[Epoch {epoch+1:03d}] Train MSE: {train_loss:.6f}, Test MSE: {test_loss:.6f}")

    writer.close()
    traced_model = torch.jit.trace(model, torch.randn(1, seq_len, input_dim).to(device))
    traced_model.save(model_name + ".pt")
    print(f"✅ TorchScript形式（{model_name}.pt）で保存しました")
    print(f"✅ TensorBoard ログを '{log_dir}' に保存しました")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('csv_path', type=str, help='Path to dataset CSV file')
    parser.add_argument('--batch-size', type=int, default=32, help='Batch size (default: 32)')
    parser.add_argument('--epochs', type=int, default=100, help='Number of epochs (default: 100)')
    parser.add_argument('--seq-len', type=int, default=10, help='Sequence length for LSTM (default: 10)')
    args = parser.parse_args()

    if not os.path.isfile(args.csv_path):
        print(f"CSVファイルが見つかりません: {args.csv_path}")
        sys.exit(1)

    model_name = input("Output model name is :")

    train_model(csv_path=args.csv_path, num_epochs=args.epochs, batch_size=args.batch_size, model_name=model_name, seq_len=args.seq_len)
