import torch
from torch import nn

from Models.model_components.common_layers import Conv

HEAD_MODES = ('argmax', 'dist')


def soft_argmax(dist_logits):
    num_cols = dist_logits.shape[-1]
    weights = dist_logits.softmax(dim=-1)
    columns = torch.arange(num_cols, device=dist_logits.device, dtype=dist_logits.dtype)
    return (weights * columns).sum(-1) / (num_cols - 1)  # ラベル規約 u/(width-1) に合わせる


class Head(nn.Module):
    def __init__(self, in_ch, num_slots, hidden=32, mode='argmax'):
        super().__init__()
        if mode not in HEAD_MODES:
            raise ValueError(f'未知の mode: {mode!r}（有効値: {HEAD_MODES}）')
        self.mode = mode
        mid = in_ch // 2

        self.v1 = Conv(in_ch, mid, nn.SiLU(), k=(2, 1), s=(2, 1))
        self.v2 = Conv(in_ch, mid, nn.SiLU(), k=1)

        self.c1 = Conv(mid * 2, hidden, nn.SiLU(), k=3, p=1)
        self.dist_conv = nn.Conv2d(hidden, num_slots, kernel_size=1)
        self.c2 = Conv(mid * 2, hidden, nn.SiLU(), k=3, p=1)
        self.valid_conv = nn.Conv1d(hidden * 2, num_slots, kernel_size=3, padding=1)

        self.exist_fc = nn.Linear(hidden * 2, num_slots)

    def forward(self, x):
        p2, p3 = x
        rows = self.v1(p2)

        coarse = nn.functional.interpolate(self.v2(p3), size=rows.shape[2:], mode='nearest')
        features = torch.cat((rows, coarse), dim=1)

        dist_logits = self.dist_conv(self.c1(features))

        v = self.c2(features)
        valid = self.valid_conv(torch.cat((v.amax(dim=-1), v.mean(dim=-1)), dim=1))
        exist = self.exist_fc(torch.cat((v.amax(dim=(2, 3)), v.mean(dim=(2, 3))), dim=1))

        if self.mode == 'argmax':
            return exist, valid, soft_argmax(dist_logits)

        return exist, valid, dist_logits


def decode_positions(raw, window=5):
    if raw.dim() == 3:
        return raw  # argmax モードは Head が正規化位置まで潰して返すので素通し

    dist = raw
    num_cols = dist.shape[-1]
    peak = dist.argmax(dim=-1, keepdim=True)

    offset = torch.arange(-window, window + 1, device=dist.device)
    index_raw = peak + offset
    inside = (index_raw >= 0) & (index_raw < num_cols)
    index = index_raw.clamp(0, num_cols - 1)

    logit = dist.gather(-1, index).masked_fill(~inside, torch.finfo(dist.dtype).min)
    weight = logit.softmax(dim=-1)
    position = (weight * index.float()).sum(dim=-1)

    return position / (num_cols - 1)
