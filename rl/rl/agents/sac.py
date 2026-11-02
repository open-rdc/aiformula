import torch
import torch.optim as optim
import torch.nn.functional as F
from my_rl_pkg.agents.networks import Actor, Critic

class SACAgent:
    def __init__(
        self,
        in_channels: int,
        action_dim: int,
        device: torch.device,
        lr: float = 3e-4,
        gamma: float = 0.99,
        tau: float = 0.005,
        alpha: float = 0.2,
        automatic_entropy_tuning: bool = True
    ):
        self.device = device
        self.gamma = gamma
        self.tau = tau

        # ネットワークの初期化
        self.actor = Actor(in_channels, action_dim).to(device)
        self.critic = Critic(in_channels, action_dim).to(device)
        self.critic_target = Critic(in_channels, action_dim).to(device)
        self.critic_target.load_state_dict(self.critic.state_dict())

        # オプティマイザ
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=lr)

        # エントロピー（Alpha）の自動調整設定
        self.automatic_entropy_tuning = automatic_entropy_tuning
        if self.automatic_entropy_tuning:
            # 目標エントロピーは行動次元のマイナス値にするのが一般的 (-action_dim)
            self.target_entropy = -float(action_dim)
            self.log_alpha = torch.zeros(1, requires_grad=True, device=device)
            self.alpha_optimizer = optim.Adam([self.log_alpha], lr=lr)
            self.alpha = self.log_alpha.exp()
        else:
            self.alpha = alpha

    def select_action(self, state: np.ndarray, evaluate: bool = False) -> np.ndarray:
        """推論時・収集時のアクション選択"""
        # numpy配列 (C, H, W) -> Tensor (1, C, H, W) [0, 1] に正規化
        state = torch.FloatTensor(state).unsqueeze(0).to(self.device) / 255.0
        
        with torch.no_grad():
            if evaluate:
                _, _, action = self.actor.sample(state)
            else:
                action, _, _ = self.actor.sample(state)
        
        return action.cpu().numpy()[0]

    def update_parameters(self, memory, batch_size: int):
        """リプレイバッファからミニバッチをサンプリングしてネットワークを更新"""
        state, action, next_state, reward, not_done = memory.sample(batch_size, self.device)

        with torch.no_grad():
            # 次の状態でのアクションと対数確率をサンプリング
            next_state_action, next_state_log_pi, _ = self.actor.sample(next_state)
            
            # ターゲットCriticによるQ値の計算（Twin）
            target_q1, target_q2 = self.critic_target(next_state, next_state_action)
            target_q = torch.min(target_q1, target_q2) - self.alpha * next_state_log_pi
            
            # ベルマン方程式に基づくターゲットQ値の算出
            target_q = reward + not_done * self.gamma * target_q

        # 現在のQ値を取得
        current_q1, current_q2 = self.critic(state, action)
        
        # Critic損失関数
        critic_loss = F.mse_loss(current_q1, target_q) + F.mse_loss(current_q2, target_q)

        # Critic更新
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # Actor損失関数の計算
        pi, log_pi, _ = self.actor.sample(state)
        actor_q1, actor_q2 = self.critic(state, pi)
        actor_q = torch.min(actor_q1, actor_q2)
        
        actor_loss = (self.alpha * log_pi - actor_q).mean()

        # Actor更新
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # エントロピー係数（Alpha）の自動調整（有効な場合）
        if self.automatic_entropy_tuning:
            alpha_loss = -(self.log_alpha.exp() * (log_pi + self.target_entropy).detach()).mean()

            self.alpha_optimizer.zero_grad()
            alpha_loss.backward()
            self.alpha_optimizer.step()

            self.alpha = self.log_alpha.exp()
            alpha_loss_val = alpha_loss.item()
        else:
            alpha_loss_val = 0.0

        # ターゲットネットワークのsoft更新
        for param, target_param in zip(self.critic.parameters(), self.critic_target.parameters()):
            target_param.data.copy_(self.tau * param.data + (1.0 - self.tau) * target_param.data)

        return critic_loss.item(), actor_loss.item(), alpha_loss_val