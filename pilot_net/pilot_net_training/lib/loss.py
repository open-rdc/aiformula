import torch
import torch.nn as nn


class WeightedSmoothL1Loss(nn.Module):
    def __init__(self, weights: list[float] | None = None, beta: float = 1.0):
        super().__init__()
        self.weights = torch.tensor(weights) if weights is not None else None
        self.smooth_l1 = nn.SmoothL1Loss(beta=beta, reduction='none')

    def forward(self, pred: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
        loss = self.smooth_l1(pred, target)
        if self.weights is not None:
            loss = loss * self.weights.to(loss.device)
        return loss.mean()


def build_loss(name: str, **kwargs) -> nn.Module:
    if name == 'mse':
        return nn.MSELoss()
    if name == 'weighted_smooth_l1':
        return WeightedSmoothL1Loss(**kwargs)
    raise ValueError(f'unknown loss: {name}')
