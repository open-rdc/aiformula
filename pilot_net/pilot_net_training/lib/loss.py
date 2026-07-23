import torch
import torch.nn as nn


class WeightedMSELoss(nn.Module):
    def __init__(self, steer_weight: float = 1.0, linear_weight: float = 1.0):
        super().__init__()
        self.register_buffer('weights', torch.tensor([steer_weight, linear_weight]))
        self.mse = nn.MSELoss(reduction='none')

    def forward(self, pred: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
        loss = self.mse(pred, target)
        return (loss * self.weights).mean()


def build_loss(steer_weight: float = 1.0, linear_weight: float = 1.0) -> nn.Module:
    return WeightedMSELoss(steer_weight=steer_weight, linear_weight=linear_weight)
