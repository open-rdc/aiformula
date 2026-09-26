import torch
import torch.nn.functional as F

from Models.data_utils.vision_planner.load_data_vision_planner import SEPARATION_THRESHOLD


def compute_loss(outputs, targets, branch_scale, smooth_scale):
    valid, position = outputs
    target_valid, target_xp = targets

    branch_diverged = (target_xp[:, 1:] - target_xp[:, :1]).abs() > SEPARATION_THRESHOLD
    diverged = torch.cat((torch.zeros_like(target_valid[:, :1]), branch_diverged.float()), dim=1)
    # straight が無効な行は target_xp[:, :1] が 0.0 のプレースホルダで比較が成立しない
    diverged = diverged * target_valid * target_valid[:, :1]
    shared = target_valid - diverged
    scale = branch_scale * shared.sum() / diverged.sum().clamp(min=1.0)
    weight = shared + scale * diverged

    error = (position - target_xp).abs() * weight
    loss_xp = error.sum() / weight.sum()
    loss_valid = F.binary_cross_entropy_with_logits(valid, target_valid)

    curvature = position[..., 2:] - 2 * position[..., 1:-1] + position[..., :-2]
    mask = target_valid[..., 2:] * target_valid[..., 1:-1] * target_valid[..., :-2]
    loss_smooth = (curvature.abs() * mask).sum() / mask.sum().clamp(min=1.0)

    total = loss_xp + loss_valid + smooth_scale * loss_smooth
    parts = {'xp': float(loss_xp), 'valid': float(loss_valid), 'smooth': float(loss_smooth)}

    return total, parts
