import torch
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
from typing import Optional
from rl.net.network import Actor, Critic

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

        self.actor = Actor(in_channels, action_dim).to(device)
        self.critic = Critic(in_channels, action_dim).to(device)
        self.critic_target = Critic(in_channels, action_dim).to(device)
        self.critic_target.load_state_dict(self.critic.state_dict())

        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=lr)

        self.automatic_entropy_tuning = automatic_entropy_tuning
        if self.automatic_entropy_tuning:
            self.target_entropy = -float(action_dim)
            self.log_alpha = torch.zeros(1, requires_grad=True, device=device)
            self.alpha_optimizer = optim.Adam([self.log_alpha], lr=lr)
            self.alpha = self.log_alpha.exp()
        else:
            self.alpha = alpha

    def checkpoint(self) -> dict:
        ckpt = {
            'actor': self.actor.state_dict(),
            'critic': self.critic.state_dict(),
            'critic_target': self.critic_target.state_dict(),
            'actor_optimizer': self.actor_optimizer.state_dict(),
            'critic_optimizer': self.critic_optimizer.state_dict(),
        }
        if self.automatic_entropy_tuning:
            ckpt['log_alpha'] = self.log_alpha.detach().cpu()
            ckpt['alpha_optimizer'] = self.alpha_optimizer.state_dict()
        return ckpt

    def load_checkpoint(self, ckpt: dict):
        self.actor.load_state_dict(ckpt['actor'])
        self.critic.load_state_dict(ckpt['critic'])
        self.critic_target.load_state_dict(ckpt['critic_target'])
        self.actor_optimizer.load_state_dict(ckpt['actor_optimizer'])
        self.critic_optimizer.load_state_dict(ckpt['critic_optimizer'])
        if self.automatic_entropy_tuning and 'log_alpha' in ckpt:
            with torch.no_grad():
                self.log_alpha.copy_(ckpt['log_alpha'].to(self.device))
            self.alpha_optimizer.load_state_dict(ckpt['alpha_optimizer'])
            self.alpha = self.log_alpha.exp()

    def load_networks(self, actor_state: dict, critic_state: Optional[dict] = None):
        self.actor.load_state_dict(actor_state)
        if critic_state is not None:
            self.critic.load_state_dict(critic_state)
            self.critic_target.load_state_dict(critic_state)

    def select_action(self, state: np.ndarray, evaluate: bool = False) -> np.ndarray:
        state = torch.FloatTensor(state).unsqueeze(0).to(self.device) / 255.0
        
        with torch.no_grad():
            if evaluate:
                _, _, action = self.actor.sample(state)
            else:
                action, _, _ = self.actor.sample(state)
        
        return action.cpu().numpy()[0]

    def update_parameters(self, memory, batch_size: int):
        state, action, next_state, reward, not_done = memory.sample(batch_size, self.device)

        with torch.no_grad():
            next_state_action, next_state_log_pi, _ = self.actor.sample(next_state)
            
            target_q1, target_q2 = self.critic_target(next_state, next_state_action)
            target_q = torch.min(target_q1, target_q2) - self.alpha * next_state_log_pi
            
            target_q = reward + not_done * self.gamma * target_q

        current_q1, current_q2 = self.critic(state, action)
        
        critic_loss = F.mse_loss(current_q1, target_q) + F.mse_loss(current_q2, target_q)

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        pi, log_pi, _ = self.actor.sample(state)
        actor_q1, actor_q2 = self.critic(state, pi)
        actor_q = torch.min(actor_q1, actor_q2)
        
        actor_loss = (self.alpha * log_pi - actor_q).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        if self.automatic_entropy_tuning:
            alpha_loss = -(self.log_alpha.exp() * (log_pi + self.target_entropy).detach()).mean()

            self.alpha_optimizer.zero_grad()
            alpha_loss.backward()
            self.alpha_optimizer.step()

            self.alpha = self.log_alpha.exp()
            alpha_loss_val = alpha_loss.item()
        else:
            alpha_loss_val = 0.0

        for param, target_param in zip(self.critic.parameters(), self.critic_target.parameters()):
            target_param.data.copy_(self.tau * param.data + (1.0 - self.tau) * target_param.data)

        return critic_loss.item(), actor_loss.item(), alpha_loss_val