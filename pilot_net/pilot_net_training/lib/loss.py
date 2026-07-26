import torch
import torch.nn as nn


class WeightedMSELoss(nn.Module):
    def __init__(self):
        super().__init__()
        self.mse = nn.MSELoss(reduction='none')

    def forward(self, pred: torch.Tensor, target: torch.Tensor,
                sample_weight: torch.Tensor = None) -> torch.Tensor:
        per_sample = self.mse(pred, target).mean(dim=1)
        if sample_weight is None:
            return per_sample.mean()
        return (per_sample * sample_weight).mean()


def build_loss() -> nn.Module:
    return WeightedMSELoss()
