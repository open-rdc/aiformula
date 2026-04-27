import torch
import torch.nn as nn


class Network(nn.Module):
    def __init__(self, num_waypoints: int = 10):
        super(Network, self).__init__()

        self.conv1 = nn.Conv2d(1, 32, kernel_size=8, stride=4)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=2)
        self.conv3 = nn.Conv2d(64, 64, kernel_size=3, stride=1)
        self.fc1 = nn.Linear(960, 512)
        self.fc2 = nn.Linear(512, num_waypoints * 2)
        self.flatten = nn.Flatten()

        self.relu = nn.ReLU()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.relu(self.conv1(x))
        x = self.relu(self.conv2(x))
        x = self.relu(self.conv3(x))
        x = self.flatten(x)
        x = self.relu(self.fc1(x))
        x = self.fc2(x)

        return x


class JunctionClassifier(nn.Module):
    """
    二値マスク画像から分岐方向を3クラス分類するネットワーク。

    入力: 1×48×64  (YOLOP二値マスク)
    出力: 3ロジット  (0=道なり, 1=右折, 2=左折)

    conv層はNetworkと同じ構造 (入力 → 960次元特徴) を共有し、
    分類ヘッドのみ変更している。
    """

    def __init__(self) -> None:
        super().__init__()

        # Input 1×48×64
        # conv1: → 32×11×15
        self.conv1 = nn.Conv2d(1, 32, kernel_size=8, stride=4)
        # conv2: → 64×5×7
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=2)
        # conv3: → 64×3×5  →  flatten: 960
        self.conv3 = nn.Conv2d(64, 64, kernel_size=3, stride=1)
        self.flatten = nn.Flatten()

        self.fc1 = nn.Linear(960, 256)
        self.fc2 = nn.Linear(256, 3)
        self.relu = nn.ReLU()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.relu(self.conv1(x))
        x = self.relu(self.conv2(x))
        x = self.relu(self.conv3(x))
        x = self.flatten(x)
        x = self.relu(self.fc1(x))
        return self.fc2(x)
