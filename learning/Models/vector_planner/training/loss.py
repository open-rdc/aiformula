import torch
import torch.nn.functional as F
from scipy.optimize import linear_sum_assignment

from Models.vector_planner.model.heads import AGENT_NO_OBJECT, box_giou, match_agents

NO_OBJECT = 3
NO_OBJECT_WEIGHT = 0.1
MATCH_POINT_WEIGHT = 5.0
CLEARANCE_MARGIN_M = 0.15
UNOBSERVABLE_WEIGHT = 0.2
DEFAULT_WEIGHTS = {'map_cls': 1.0, 'map_points': 5.0, 'agent_cls': 1.0, 'agent_l1': 5.0, 'agent_giou': 2.0,
                   'path': 1.0, 'path_valid': 0.1, 'clearance': 1.0, 'corridor': 1.0, 'branch_exist': 1.0,
                   'boundary': 1.0, 'direction': 0.2, 'agent_clearance': 0.2, 'signal': 1.0,
                   'avoid_weight': 1.0, 'tread': 0.6}
SIGNAL_CLASS_WEIGHTS = (0.3, 2.0, 1.0, 1.0)
SIGNAL_X_TOL_RATE, SIGNAL_Y_WEIGHT = 0.15, 0.5
AGENT_NO_OBJECT_WEIGHT = 0.1
AGENT_RADIUS_M = 0.15
AGENT_SCORE_THRESHOLD = 0.3
DIRECTION_COS_MIN = 0.95
EPS = 1e-9
IGNORE_INDEX = -100


def _geo_mask(batch, size):
    # 検出専用フレーム（has_geo=False）は地図/分岐の損失から外す
    if 'has_geo' in batch:
        return batch['has_geo'].bool()
    return torch.ones(size, dtype=torch.bool, device=batch['map_mask'].device)


def map_loss(outputs, batch):
    logits, points = outputs['map_logits'], outputs['map_points']
    b, q, _ = logits.shape
    target_cls = torch.full((b, q), NO_OBJECT, dtype=torch.long, device=logits.device)
    matched_pred, matched_gt = [], []
    prob = logits.detach().softmax(-1)
    geo = _geo_mask(batch, b)
    for i in range(b):
        gt = batch['map_mask'][i].nonzero().squeeze(1)
        if len(gt) == 0 and not bool(geo[i]):
            target_cls[i] = IGNORE_INDEX
        if len(gt) == 0:
            continue
        gt_points = batch['map_points'][i, gt]
        gt_cls = batch['map_cls'][i, gt]
        point_cost = torch.cdist(points[i].detach().flatten(1), gt_points.flatten(1), p=1) / gt_points[0].numel()
        cost = MATCH_POINT_WEIGHT * point_cost - prob[i][:, gt_cls]
        if not bool(torch.isfinite(cost).all()):
            raise FloatingPointError('map マッチングのコストが非有限（モデル出力に NaN/inf）')
        rows, cols = linear_sum_assignment(cost.cpu().numpy())
        rows, cols = torch.as_tensor(rows, device=logits.device), torch.as_tensor(cols, device=logits.device)
        target_cls[i, rows] = gt_cls[cols]
        matched_pred.append(points[i, rows])
        matched_gt.append(gt_points[cols])

    weight = torch.tensor([1.0, 1.0, 1.0, NO_OBJECT_WEIGHT], device=logits.device)
    cls_loss = F.cross_entropy(logits.reshape(-1, 4), target_cls.reshape(-1), weight=weight, ignore_index=IGNORE_INDEX)
    if not bool((target_cls != IGNORE_INDEX).any()):
        cls_loss = logits.sum() * 0.0
    if matched_pred:
        point_loss = (torch.cat(matched_pred) - torch.cat(matched_gt)).abs().mean()
    else:
        point_loss = logits.sum() * 0.0
    return cls_loss, point_loss


def path_losses(outputs, batch, avoid_weight=1.0):
    mask = batch['path_mask'].to(outputs['path_xy'].dtype)
    weight = torch.ones_like(mask)
    if avoid_weight != 1.0 and 'path_avoid' in batch:
        weight = weight + (avoid_weight - 1.0) * batch['path_avoid'].to(mask.dtype)[:, :, None]
    diff = (outputs['path_xy'] - batch['path_xy']).abs() * (mask * weight)[..., None]
    path = diff.sum() / (2 * mask.sum()).clamp(min=1.0)

    slot = batch['path_slot_ok'].to(mask.dtype)[:, :, None].expand_as(mask)
    bce = F.binary_cross_entropy_with_logits(outputs['path_valid'], batch['path_valid'].to(mask.dtype), reduction='none')
    valid = (bce * slot).sum() / slot.sum().clamp(min=1.0)
    return path, valid


def _distance_to_polyline(path, point):
    # path (B,N,2)、point (B,P,2) -> (B,P)。原点始まりの折れ線までの最短距離
    origin = torch.zeros_like(path[:, :1])
    line = torch.cat([origin, path], dim=1)
    a, b = line[:, None, :-1], line[:, None, 1:]
    seg = b - a
    length_sq = (seg ** 2).sum(-1).clamp(min=1e-9)
    t = (((point[:, :, None] - a) * seg).sum(-1) / length_sq).clamp(0.0, 1.0)
    foot = a + t[..., None] * seg
    return ((point[:, :, None] - foot) ** 2).sum(-1).add(1e-12).sqrt().min(dim=-1).values


def clearance_loss(outputs, batch, tread=0.6):
    # straight が avoid のサンプルで、予測経路と貼り付けコーンの距離が足りない分のヒンジ
    path = outputs['path_xy'][:, 0]
    pylons, active = batch['pylons'], batch['pylon_mask'] & batch['path_avoid'][:, 0:1]
    distance = _distance_to_polyline(path, pylons[..., :2])
    threshold = tread / 2.0 + pylons[..., 2] / 2.0 + CLEARANCE_MARGIN_M
    hinge = F.relu(threshold - distance) * active.to(distance.dtype)
    return hinge.sum() / active.sum().clamp(min=1)


def corridor_loss(outputs, batch):
    mask = batch['corridor_mask'].to(outputs['corridor_left'].dtype)
    weight = mask * torch.where(batch['corridor_obs'], 1.0, UNOBSERVABLE_WEIGHT)
    err = ((outputs['corridor_left'] - batch['corridor_left']).abs()
           + (outputs['corridor_right'] - batch['corridor_right']).abs()).sum(-1)
    return (err * weight).sum() / (4 * weight.sum()).clamp(min=1e-6)


def agent_losses(outputs, batch):
    # DETR 式。has_boxes のサンプルだけ、ハンガリアン割当で CE / L1 / (1 - GIoU)
    logits, boxes = outputs['agent_logits'], outputs['agent_boxes']
    zero = logits.sum() * 0.0
    use = batch['has_boxes'].nonzero().squeeze(1)
    if len(use) == 0:
        return zero, zero, zero
    logits, boxes = logits[use], boxes[use]
    sub = {k: batch[k][use] for k in ('boxes', 'box_cls', 'box_mask', 'has_boxes')}
    pairs = match_agents(logits, boxes, sub)
    target = torch.full(logits.shape[:2], AGENT_NO_OBJECT, dtype=torch.long, device=logits.device)
    matched_pred, matched_gt = [], []
    for i, (rows, cols) in enumerate(pairs):
        if len(rows):
            target[i, rows] = sub['box_cls'][i, cols]
            matched_pred.append(boxes[i, rows])
            matched_gt.append(sub['boxes'][i, cols])
    weight = torch.ones(AGENT_NO_OBJECT + 1, device=logits.device)
    weight[AGENT_NO_OBJECT] = AGENT_NO_OBJECT_WEIGHT
    cls = F.cross_entropy(logits.reshape(-1, AGENT_NO_OBJECT + 1), target.reshape(-1), weight=weight)
    if not matched_pred:
        return cls, zero, zero
    pred, gt = torch.cat(matched_pred), torch.cat(matched_gt)
    return cls, (pred - gt).abs().mean(), (1.0 - box_giou(pred, gt)).mean()


def signal_loss(outputs, batch):
    # CE（ignore=-100 は除外）と、x を距離で正規化した L1。xy は signal_xy_mask の要素だけ
    logits, xy = outputs['signal_logits'], outputs['signal_xy']
    zero = logits.sum() * 0.0 + xy.sum() * 0.0
    target = batch['signal_cls'].to(logits.device)
    weight = torch.tensor(SIGNAL_CLASS_WEIGHTS, dtype=logits.dtype, device=logits.device)
    keep = target != IGNORE_INDEX
    if bool(keep.any()):
        cls = F.cross_entropy(logits, target, weight=weight, ignore_index=IGNORE_INDEX)
    else:
        cls = zero
    mask = batch['signal_xy_mask'].to(logits.device)
    rows = mask.any(-1)
    if not bool(rows.any()):
        return cls, zero
    gt = batch['signal_xy'].to(xy.dtype).to(xy.device)
    scale = (SIGNAL_X_TOL_RATE * gt[:, 0].abs()).clamp(min=1.0)
    err_x = (xy[:, 0] - gt[:, 0]).abs() / scale
    err_y = SIGNAL_Y_WEIGHT * (xy[:, 1] - gt[:, 1]).abs()
    per = err_x * mask[:, 0] + err_y * mask[:, 1]
    return cls, per.sum() / rows.sum().clamp(min=1)


def branch_exist_loss(outputs, batch):
    logits = outputs['branch_exist']
    geo = _geo_mask(batch, logits.shape[0])
    if not bool(geo.any()):
        return logits.sum() * 0.0
    return F.binary_cross_entropy_with_logits(logits[geo], batch['branch_exist'][geo].to(logits.dtype))


def _gt_corridor_frame(batch):
    center = (batch['corridor_left'] + batch['corridor_right']) / 2
    half = (batch['corridor_left'] - batch['corridor_right']) / 2
    return center, half


def boundary_loss(outputs, batch):
    # 予測経路が GT の観測済み走行帯境界を越えた距離のヒンジ
    center, half = _gt_corridor_frame(batch)
    half_len = (half ** 2).sum(-1).add(EPS).sqrt()
    normal = half / half_len[..., None]
    offset = ((outputs['path_xy'] - center) * normal).sum(-1).abs()
    mask = (batch['corridor_mask'] & batch['corridor_obs']).to(offset.dtype)
    return (F.relu(offset - half_len) * mask).sum() / mask.sum().clamp(min=1.0)


def direction_loss(outputs, batch, cos_min=DIRECTION_COS_MIN):
    # 予測経路の接線と GT 走行帯中心線の接線の cos が低い分のペナルティ
    center, _ = _gt_corridor_frame(batch)
    pred = outputs['path_xy'][..., 1:, :] - outputs['path_xy'][..., :-1, :]
    gt = center[..., 1:, :] - center[..., :-1, :]
    dot = (pred * gt).sum(-1)
    norm = (pred ** 2).sum(-1).add(EPS).sqrt() * (gt ** 2).sum(-1).add(EPS).sqrt()
    mask = (batch['corridor_mask'][..., 1:] & batch['corridor_mask'][..., :-1]).to(dot.dtype)
    return (F.relu(cos_min - dot / norm) * mask).sum() / mask.sum().clamp(min=1.0)


def agent_clearance_loss(outputs, batch, tread=0.6, score_threshold=AGENT_SCORE_THRESHOLD):
    # straight が avoid のサンプルで、確信度の高い推論 agent（座標は detach）と予測経路の距離のヒンジ
    score = outputs['agent_logits'].detach().softmax(-1)[..., :AGENT_NO_OBJECT].max(-1).values
    active = (score >= score_threshold) & batch['path_avoid'][:, 0:1]
    distance = _distance_to_polyline(outputs['path_xy'][:, 0], outputs['agent_ground_xy'].detach())
    threshold = tread / 2.0 + AGENT_RADIUS_M + CLEARANCE_MARGIN_M
    hinge = F.relu(threshold - distance) * active.to(distance.dtype)
    return hinge.sum() / active.sum().clamp(min=1)


def planner_loss(outputs, batch, weights=None):
    w = dict(DEFAULT_WEIGHTS)
    w.update(weights or {})
    parts = {}
    if w['map_cls'] > 0 or w['map_points'] > 0:
        parts['map_cls'], parts['map_points'] = map_loss(outputs, batch)
    if w['agent_cls'] > 0 or w['agent_l1'] > 0 or w['agent_giou'] > 0:
        parts['agent_cls'], parts['agent_l1'], parts['agent_giou'] = agent_losses(outputs, batch)
    if w['path'] > 0 or w['path_valid'] > 0:
        parts['path'], parts['path_valid'] = path_losses(outputs, batch, w['avoid_weight'])
    if w['clearance'] > 0 and 'pylons' in batch and 'path_avoid' in batch:
        parts['clearance'] = clearance_loss(outputs, batch, w['tread'])
    if w['corridor'] > 0:
        parts['corridor'] = corridor_loss(outputs, batch)
    if w['branch_exist'] > 0:
        parts['branch_exist'] = branch_exist_loss(outputs, batch)
    if w['boundary'] > 0:
        parts['boundary'] = boundary_loss(outputs, batch)
    if w['direction'] > 0:
        parts['direction'] = direction_loss(outputs, batch)
    if w['agent_clearance'] > 0:
        parts['agent_clearance'] = agent_clearance_loss(outputs, batch, w['tread'])
    if 'signal_logits' in outputs and 'signal_cls' in batch and w['signal'] > 0:
        parts['signal_cls'], parts['signal_xy'] = signal_loss(outputs, batch)
        w['signal_cls'] = w['signal_xy'] = w['signal']
    parts = {k: v for k, v in parts.items() if w[k] > 0}
    if parts:
        finite = torch.stack([torch.isfinite(v) for v in parts.values()])
        if not bool(finite.all()):
            bad = [k for k, ok in zip(parts, finite.tolist()) if not ok]
            raise FloatingPointError(f'非有限の損失項: {bad}')
    total = sum(w[k] * v for k, v in parts.items()) if parts else torch.zeros(())
    return total, {k: v.detach() for k, v in parts.items()}
