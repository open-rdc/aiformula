import torch
import torch.nn as nn

class Network(nn.Module):
    def __init__(self, num_waypoints: int = 10):
        super(Network, self).__init__()

        self.conv1 = nn.Conv2d(4, 32, kernel_size=8, stride=4)
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
