import torch
from torch import nn

from Models.model_components.common_layers import Conv

ROW_START = 25  # 地平線より上は教師が付かない


def soft_argmax(dist_logits):
    num_cols = dist_logits.shape[-1]
    weights = dist_logits.softmax(dim=-1)
    columns = torch.arange(num_cols, device=dist_logits.device, dtype=dist_logits.dtype)
    return (weights * columns).sum(-1) / (num_cols - 1)


class Head(nn.Module):
    def __init__(self, in_ch, num_slots, hidden=32):
        super().__init__()
        mid = in_ch // 2

        self.v1 = Conv(in_ch, mid, nn.SiLU(), k=(2, 1), s=(2, 1))
        self.v2 = Conv(in_ch, mid, nn.SiLU(), k=1)

        self.c1 = Conv(mid * 2, hidden, nn.SiLU(), k=3, p=1)
        self.c2 = Conv(mid * 2, hidden, nn.SiLU(), k=3, p=1)
        self.dist_conv = nn.Conv2d(hidden, num_slots, kernel_size=1)
        self.valid_conv = nn.Conv1d(hidden * 2, num_slots, kernel_size=3, padding=1)

        # 分岐チャネルをゼロ初期化し、学習開始時は straight と厳密一致させる
        nn.init.zeros_(self.dist_conv.weight[1:])
        nn.init.zeros_(self.dist_conv.bias[1:])

    def forward(self, x):
        p2, p3 = x
        rows = self.v1(p2)

        coarse = nn.functional.interpolate(self.v2(p3), size=rows.shape[2:], mode='nearest')
        features = torch.cat((rows, coarse), dim=1)[:, :, ROW_START:, :]

        position_features = self.c1(features)
        raw_logits = self.dist_conv(position_features)
        dist_logits = torch.cat((raw_logits[:, :1], raw_logits[:, :1] + raw_logits[:, 1:]), dim=1)  # 分岐チャネルは straight への残差

        v = self.c2(features)
        valid = self.valid_conv(torch.cat((v.amax(dim=-1), v.mean(dim=-1)), dim=1))

        position = soft_argmax(dist_logits)
        return valid, position
