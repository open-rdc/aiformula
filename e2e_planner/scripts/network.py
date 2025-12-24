import torch
import torch.nn as nn
from torch_geometric.nn import MLP, PointNetConv, fps, global_max_pool, radius

class CNNModule(nn.Module):
    def __init__(self):
        super(CNNModule, self).__init__()

        self.conv1 = nn.Conv2d(1, 32, kernel_size=8, stride=4)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=2)
        self.conv3 = nn.Conv2d(64, 64, kernel_size=3, stride=1)
        self.flatten = nn.Flatten()

        self.relu = nn.ReLU()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.relu(self.conv1(x))
        x = self.relu(self.conv2(x))
        x = self.relu(self.conv3(x))
        x = self.flatten(x)

        return x   


class SAModule(nn.Module):
    def __init__(self, ratio, r, nn):
        super().__init__()
        self.ratio = ratio
        self.r = r
        self.conv = PointNetConv(nn, add_self_loops=False)

    def forward(self, x, pos, batch):
        idx = fps(pos, batch, ratio=self.ratio)
        row, col = radius(pos, pos[idx], self.r, batch, batch[idx],
                         max_num_neighbors=64)
        edge_index = torch.stack([col, row], dim=0)
        x_dst = None if x is None else x[idx]
        x = self.conv((x, x_dst), (pos, pos[idx]), edge_index)
        pos, batch = pos[idx], batch[idx]
        return x, pos, batch


class GlobalSAModule(nn.Module):
    def __init__(self, nn):
        super().__init__()
        self.nn = nn

    def forward(self, x, pos, batch):
        x = self.nn(torch.cat([x, pos], dim=1))
        x = global_max_pool(x, batch)
        pos = pos.new_zeros((x.size(0), 3))
        batch = torch.arange(x.size(0), device=batch.device)
        return x, pos, batch


class PointNet2Module(nn.Module):
    def __init__(self):
        super(PointNet2Module, self).__init__()

        self.sa1_module = SAModule(0.5, 0.2, MLP([3, 64, 64, 128]))
        self.sa2_module = SAModule(0.25, 0.4, MLP([128 + 3, 128, 128, 256]))
        self.sa3_module = GlobalSAModule(MLP([256 + 3, 256, 512, 1024]))

    def forward(self, pos, batch):
        x = None
        x, pos, batch = self.sa1_module(x, pos, batch)
        x, pos, batch = self.sa2_module(x, pos, batch)
        x, pos, batch = self.sa3_module(x, pos, batch)
        return x


class Network(nn.Module):
    def __init__(self, num_waypoints: int = 10):
        super(Network, self).__init__()

        self.cnn_module = CNNModule()
        self.pointnet2_module = PointNet2Module()

        #（CNN out_dim : 960) + (PointNet++ out_dim : 1024) = 1984
        self.fc1 = nn.Linear(960 + 1024, 512)
        self.fc2 = nn.Linear(512, num_waypoints * 2)

        self.relu = nn.ReLU()

    def forward(self, img: torch.Tensor, pos: torch.Tensor, batch: torch.Tensor) -> torch.Tensor:

        cnn_features = self.cnn_module(img)
        pointnet_features = self.pointnet2_module(pos, batch)

        combined_features = torch.cat([cnn_features, pointnet_features], dim=1)

        x = self.relu(self.fc1(combined_features))
        x = self.fc2(x)

        return x
