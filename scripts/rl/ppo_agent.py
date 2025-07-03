#!/usr/bin/env python3

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.distributions import Normal
import numpy as np
from typing import Tuple, List

class ActorCritic(nn.Module):
    """PPO用のActor-Criticネットワーク"""
    
    def __init__(self, state_dim: int = 5, action_dim: int = 2, hidden_dim: int = 256):
        super().__init__()
        
        # 共通特徴抽出層
        self.feature_extractor = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU()
        )
        
        # Actor（方策）ネットワーク
        self.actor_mean = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(),
            nn.Linear(hidden_dim // 2, action_dim),
            nn.Tanh()  # [-1, 1]に正規化後、スケール
        )
        
        # Actor標準偏差（学習可能）
        self.actor_log_std = nn.Parameter(torch.zeros(action_dim))
        
        # Critic（価値）ネットワーク
        self.critic = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(),
            nn.Linear(hidden_dim // 2, 1)
        )
        
        # 重み初期化
        self.apply(self._init_weights)
        
    def _init_weights(self, m):
        if isinstance(m, nn.Linear):
            nn.init.orthogonal_(m.weight, 0.01)
            nn.init.constant_(m.bias, 0.0)
    
    def forward(self, state: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """前向き計算"""
        features = self.feature_extractor(state)
        
        # Actor出力
        action_mean = self.actor_mean(features) * 700.0  # [-700, 700]にスケール
        action_std = torch.exp(self.actor_log_std).expand_as(action_mean)
        
        # Critic出力
        value = self.critic(features)
        
        return action_mean, action_std, value
    
    def get_action(self, state: torch.Tensor, deterministic: bool = False) -> Tuple[torch.Tensor, torch.Tensor]:
        """行動選択"""
        action_mean, action_std, value = self.forward(state)
        
        if deterministic:
            action = action_mean
            log_prob = None
        else:
            dist = Normal(action_mean, action_std)
            action = dist.sample()
            log_prob = dist.log_prob(action).sum(dim=-1)
            
            # アクションをクリップ
            action = torch.clamp(action, -700.0, 700.0)
        
        return action, log_prob
    
    def evaluate_action(self, state: torch.Tensor, action: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """行動評価（PPO更新用）"""
        action_mean, action_std, value = self.forward(state)
        
        dist = Normal(action_mean, action_std)
        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        
        return log_prob, value.squeeze(), entropy

class PPOAgent:
    """PPOエージェント"""
    
    def __init__(self, state_dim: int = 5, action_dim: int = 2, lr: float = 3e-4, 
                 gamma: float = 0.99, gae_lambda: float = 0.95, clip_epsilon: float = 0.2,
                 value_coef: float = 0.5, entropy_coef: float = 0.01, max_grad_norm: float = 0.5,
                 device: str = "auto"):
        
        # デバイス選択
        if device == "auto":
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        elif device == "cuda":
            if not torch.cuda.is_available():
                print("Warning: CUDA requested but not available. Using CPU.")
                self.device = torch.device("cpu")
            else:
                self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")
        
        # ハイパーパラメータ
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.clip_epsilon = clip_epsilon
        self.value_coef = value_coef
        self.entropy_coef = entropy_coef
        self.max_grad_norm = max_grad_norm
        
        # ネットワーク
        self.actor_critic = ActorCritic(state_dim, action_dim).to(self.device)
        self.optimizer = optim.Adam(self.actor_critic.parameters(), lr=lr, eps=1e-5)
        
        # 経験バッファ
        self.reset_buffer()
        
        print(f"PPO Agent initialized - Device: {self.device}")
    
    def reset_buffer(self):
        """経験バッファをリセット"""
        self.states = []
        self.actions = []
        self.rewards = []
        self.log_probs = []
        self.values = []
        self.dones = []
    
    def select_action(self, state: np.ndarray, deterministic: bool = False) -> Tuple[np.ndarray, float]:
        """行動選択"""
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        
        with torch.no_grad():
            action, log_prob = self.actor_critic.get_action(state_tensor, deterministic)
            _, _, value = self.actor_critic.forward(state_tensor)
        
        action_np = action.cpu().numpy()[0]
        log_prob_np = log_prob.cpu().numpy()[0] if log_prob is not None else 0.0
        value_np = value.cpu().numpy()[0]
        
        return action_np, log_prob_np, value_np
    
    def store_transition(self, state: np.ndarray, action: np.ndarray, reward: float, 
                        log_prob: float, value: float, done: bool):
        """経験を保存"""
        self.states.append(state)
        self.actions.append(action)
        self.rewards.append(reward)
        self.log_probs.append(log_prob)
        self.values.append(value)
        self.dones.append(done)
    
    def compute_gae(self, next_value: float = 0.0) -> Tuple[List[float], List[float]]:
        """GAE（Generalized Advantage Estimation）計算"""
        advantages = []
        returns = []
        
        gae = 0
        for i in reversed(range(len(self.rewards))):
            if i == len(self.rewards) - 1:
                next_non_terminal = 1.0 - self.dones[i]
                next_val = next_value
            else:
                next_non_terminal = 1.0 - self.dones[i]
                next_val = self.values[i + 1]
            
            delta = self.rewards[i] + self.gamma * next_val * next_non_terminal - self.values[i]
            gae = delta + self.gamma * self.gae_lambda * next_non_terminal * gae
            
            advantages.insert(0, gae)
            returns.insert(0, gae + self.values[i])
        
        return advantages, returns
    
    def update(self, next_value: float = 0.0, update_epochs: int = 4, batch_size: int = 64) -> dict[str, float]:
        """PPO更新"""
        if len(self.states) == 0:
            return {}
        
        # GAE計算
        advantages, returns = self.compute_gae(next_value)
        
        # Tensorに変換（効率的な方法）
        states = torch.tensor(np.array(self.states), dtype=torch.float32, device=self.device)
        actions = torch.tensor(np.array(self.actions), dtype=torch.float32, device=self.device)
        old_log_probs = torch.tensor(np.array(self.log_probs), dtype=torch.float32, device=self.device)
        advantages_tensor = torch.tensor(np.array(advantages), dtype=torch.float32, device=self.device)
        returns_tensor = torch.tensor(np.array(returns), dtype=torch.float32, device=self.device)
        
        # 正規化
        advantages_tensor = (advantages_tensor - advantages_tensor.mean()) / (advantages_tensor.std() + 1e-8)
        
        # 更新ループ
        total_loss = 0
        policy_loss_total = 0
        value_loss_total = 0
        entropy_loss_total = 0
        
        dataset_size = len(self.states)
        
        for _ in range(update_epochs):
            # ミニバッチに分割
            indices = np.random.permutation(dataset_size)
            
            for start in range(0, dataset_size, batch_size):
                end = min(start + batch_size, dataset_size)
                batch_indices = indices[start:end]
                
                batch_states = states[batch_indices]
                batch_actions = actions[batch_indices]
                batch_old_log_probs = old_log_probs[batch_indices]
                batch_advantages = advantages_tensor[batch_indices]
                batch_returns = returns_tensor[batch_indices]
                
                # 現在のポリシーで評価
                log_probs, values, entropy = self.actor_critic.evaluate_action(batch_states, batch_actions)
                
                # 重要度比
                ratio = torch.exp(log_probs - batch_old_log_probs)
                
                # クリップされたサロゲート損失
                surr1 = ratio * batch_advantages
                surr2 = torch.clamp(ratio, 1.0 - self.clip_epsilon, 1.0 + self.clip_epsilon) * batch_advantages
                policy_loss = -torch.min(surr1, surr2).mean()
                
                # 価値関数損失（形状を合わせる）
                values_flat = values.view(-1) if values.dim() > 1 else values
                batch_returns_flat = batch_returns.view(-1) if batch_returns.dim() > 1 else batch_returns
                value_loss = F.mse_loss(values_flat, batch_returns_flat)
                
                # エントロピー損失
                entropy_loss = -entropy.mean()
                
                # 全体損失
                loss = policy_loss + self.value_coef * value_loss + self.entropy_coef * entropy_loss
                
                # 更新
                self.optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(self.actor_critic.parameters(), self.max_grad_norm)
                self.optimizer.step()
                
                total_loss += loss.item()
                policy_loss_total += policy_loss.item()
                value_loss_total += value_loss.item()
                entropy_loss_total += entropy_loss.item()
        
        # バッファをクリア
        self.reset_buffer()
        
        num_updates = update_epochs * (dataset_size // batch_size + (1 if dataset_size % batch_size > 0 else 0))
        
        return {
            'total_loss': total_loss / num_updates,
            'policy_loss': policy_loss_total / num_updates,
            'value_loss': value_loss_total / num_updates,
            'entropy_loss': entropy_loss_total / num_updates
        }
    
    def save(self, filepath: str):
        """モデル保存"""
        torch.save({
            'actor_critic_state_dict': self.actor_critic.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict()
        }, filepath)
    
    def load(self, filepath: str):
        """モデル読み込み"""
        checkpoint = torch.load(filepath, map_location=self.device)
        self.actor_critic.load_state_dict(checkpoint['actor_critic_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])