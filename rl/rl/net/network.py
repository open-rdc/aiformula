import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal

LOG_SIG_MAX = 2.0
LOG_SIG_MIN = -20.0

class CNNFeatureExtractor(nn.Module):
    """
    画像入力（例: 1x84x84 または 3x84x84）から特徴量を抽出するCNN
    """
    def __init__(self, in_channels: int = 1, feature_dim: int = 256):
        super(CNNFeatureExtractor, self).__init__()
        
        # Atariや一般的なRLで使われる標準的なCNNアーキテクチャ
        self.conv1 = nn.Conv2d(in_channels, 32, kernel_size=8, stride=4)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=4, stride=2)
        self.conv3 = nn.Conv2d(64, 64, kernel_size=3, stride=1)
        
        # ダミー入力で畳み後のサイズを自動計算して全結合層の入力を決める
        # 84x84 入力を想定
        self.fc = nn.Linear(64 * 7 * 7, feature_dim)

    def forward(self, x):
        # x shape: (Batch, Channels, Height, Width)
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = x.reshape(x.size(0), -1)  # shape: (Batch, 64 * 7 * 7)
        x = F.relu(self.fc(x))
        return x


class Actor(nn.Module):
    """
    画像を入力とするSAC用の確率的Actorネットワーク
    """
    def __init__(self, in_channels: int, action_dim: int, hidden_dim: int = 256):
        super(Actor, self).__init__()
        
        self.cnn = CNNFeatureExtractor(in_channels, feature_dim=hidden_dim)
        
        # 出力：平均と対数標準偏差
        self.mean_linear = nn.Linear(hidden_dim, action_dim)
        self.log_std_linear = nn.Linear(hidden_dim, action_dim)

    def forward(self, state):
        features = self.cnn(state)
        
        mean = self.mean_linear(features)
        log_std = self.log_std_linear(features)
        log_std = torch.clamp(log_std, LOG_SIG_MIN, LOG_SIG_MAX)
        
        return mean, log_std

    def sample(self, state):
        mean, log_std = self.forward(state)
        std = log_std.exp()
        normal = Normal(mean, std)
        
        x_t = normal.rsample()
        action = torch.tanh(x_t)
        
        log_prob = normal.log_prob(x_t)
        log_prob -= torch.log(1.0 - action.pow(2) + 1e-6)
        log_prob = log_prob.sum(1, keepdim=True)
        
        mean = torch.tanh(mean)
        
        return action, log_prob, mean


class Critic(nn.Module):
    """
    画像を入力とするSAC用のTwin Criticネットワーク
    画像特徴量とアクションを結合してQ値を評価する
    """
    def __init__(self, in_channels: int, action_dim: int, hidden_dim: int = 256):
        super(Critic, self).__init__()
        
        # Q1用
        self.cnn_q1 = CNNFeatureExtractor(in_channels, feature_dim=hidden_dim)
        self.fc1_q1 = nn.Linear(hidden_dim + action_dim, hidden_dim)
        self.q1_out = nn.Linear(hidden_dim, 1)

        # Q2用
        self.cnn_q2 = CNNFeatureExtractor(in_channels, feature_dim=hidden_dim)
        self.fc1_q2 = nn.Linear(hidden_dim + action_dim, hidden_dim)
        self.q2_out = nn.Linear(hidden_dim, 1)

    def forward(self, state, action):
        # Q1の計算
        feat_q1 = self.cnn_q1(state)
        sa_q1 = torch.cat([feat_q1, action], dim=1)
        q1 = F.relu(self.fc1_q1(sa_q1))
        q1 = self.q1_out(q1)

        # Q2の計算
        feat_q2 = self.cnn_q2(state)
        sa_q2 = torch.cat([feat_q2, action], dim=1)
        q2 = F.relu(self.fc1_q2(sa_q2))
        q2 = self.q2_out(q2)

        return q1, q2