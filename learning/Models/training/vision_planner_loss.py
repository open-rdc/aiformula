import torch
from torch.nn import functional


def gaussian_target(target_xp, num_cols, sigma):
    centre = target_xp.unsqueeze(-1) * (num_cols - 1)
    columns = torch.arange(num_cols, device=target_xp.device, dtype=target_xp.dtype)
    logits = -(columns - centre) ** 2 / (2.0 * sigma ** 2)
    return logits.softmax(dim=-1)


def _slot_pos_weight(value, num_slots, device, dtype, name, num_trailing_dims):
    if isinstance(value, (list, tuple)):
        if len(value) != num_slots:
            raise ValueError(f'{name} の要素数({len(value)})がスロット数({num_slots})と一致しません: {value}')
        weight = torch.tensor(value, device=device, dtype=dtype)
        return weight.view(*weight.shape, *([1] * num_trailing_dims))
    return torch.tensor(float(value), device=device, dtype=dtype)


class ComputeLoss:
    def __init__(self, params):
        self.exist_gain = float(params['exist'])
        self.valid_gain = float(params['valid'])
        self.dist_gain = float(params['dist'])

        self.xp_gain = float(params.get('xp', 0.0))
        self.sigma = float(params['sigma'])

        self.valid_pos_weight = params.get('valid_pos_weight', 1.0)
        self.exist_pos_weight = params.get('exist_pos_weight', 1.0)

    def __call__(self, outputs, targets):
        exist, valid, xp_or_dist = outputs
        target_exist, target_valid, target_xp = targets
        num_slots = exist.shape[-1]

        exist_pos_weight = _slot_pos_weight(self.exist_pos_weight, num_slots, exist.device, exist.dtype,
                                            'exist_pos_weight', num_trailing_dims=0)
        loss_exist = functional.binary_cross_entropy_with_logits(exist, target_exist, pos_weight=exist_pos_weight)

        pos_weight = _slot_pos_weight(self.valid_pos_weight, num_slots, valid.device, valid.dtype,
                                      'valid_pos_weight', num_trailing_dims=1)
        loss_valid = functional.binary_cross_entropy_with_logits(valid, target_valid, pos_weight=pos_weight)

        if xp_or_dist.dim() == 3:
            position = xp_or_dist

            denominator = target_valid.sum().clamp_min(1.0)
            loss_xp = ((position - target_xp).abs() * target_valid).sum() / denominator

            total = self.exist_gain * loss_exist + self.valid_gain * loss_valid + self.xp_gain * loss_xp
            return total, {'exist': float(loss_exist), 'valid': float(loss_valid), 'xp': float(loss_xp)}

        dist = xp_or_dist
        target = gaussian_target(target_xp, dist.shape[-1], self.sigma)

        divergence = (target * (torch.log(target.clamp_min(1e-12)) - dist.log_softmax(dim=-1))).sum(dim=-1)
        denominator = target_valid.sum().clamp_min(1.0)
        loss_dist = (divergence * target_valid).sum() / denominator

        total = self.exist_gain * loss_exist + self.valid_gain * loss_valid + self.dist_gain * loss_dist
        return total, {'exist': float(loss_exist), 'valid': float(loss_valid), 'dist': float(loss_dist)}
