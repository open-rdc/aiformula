import numpy as np
from scipy.spatial import cKDTree

DENSE_STEP_M = 0.25


def path_error_sums(pred, gt, mask):
    diff = (pred - gt).abs()
    m = mask.to(diff.dtype)
    return {'l1': float((diff.sum(dim=-1) * m).sum()), 'lateral': float((diff[..., 1] * m).sum()),
            'count': int(mask.sum())}


def densify_polyline(points, step=DENSE_STEP_M):
    points = np.asarray(points, float)
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))])
    if arc[-1] < 1e-9:
        return points[:1]
    samples = np.unique(np.concatenate([np.arange(0.0, arc[-1], step), [arc[-1]]]))
    return np.stack([np.interp(samples, arc, points[:, 0]), np.interp(samples, arc, points[:, 1])], axis=1)


def _stack(polylines):
    if not polylines:
        return np.zeros((0, 2))
    return np.vstack([densify_polyline(p) for p in polylines])


def vectormap_scores(pred_polylines, gt_polylines, radius=1.0):
    pred, gt = _stack(pred_polylines), _stack(gt_polylines)
    result = {'chamfer': None, 'precision_hits': 0, 'precision_total': len(pred),
              'recall_hits': 0, 'recall_total': len(gt)}
    if len(pred) == 0 or len(gt) == 0:
        return result
    d_pred = cKDTree(gt).query(pred)[0]
    d_gt = cKDTree(pred).query(gt)[0]
    result['chamfer'] = float(0.5 * (d_pred.mean() + d_gt.mean()))
    result['precision_hits'] = int((d_pred <= radius).sum())
    result['recall_hits'] = int((d_gt <= radius).sum())
    return result


def iou(a, b):
    ax0, ay0, ax1, ay1 = a[0] - a[2] / 2, a[1] - a[3] / 2, a[0] + a[2] / 2, a[1] + a[3] / 2
    bx0, by0, bx1, by1 = b[0] - b[2] / 2, b[1] - b[3] / 2, b[0] + b[2] / 2, b[1] + b[3] / 2
    inter = max(0.0, min(ax1, bx1) - max(ax0, bx0)) * max(0.0, min(ay1, by1) - max(ay0, by0))
    union = a[2] * a[3] + b[2] * b[3] - inter
    return float(inter / union) if union > 0 else 0.0


def average_precision(gt_boxes, gt_classes, predictions, cls, iou_thr=0.5):
    gts = [[b for b, c in zip(boxes, classes) if c == cls] for boxes, classes in zip(gt_boxes, gt_classes)]
    total = sum(len(g) for g in gts)
    if total == 0:
        return None
    detections = sorted(((p[4], i, p[:4]) for i, preds in enumerate(predictions) for p in preds if p[5] == cls),
                        key=lambda t: -t[0])
    used = [np.zeros(len(g), bool) for g in gts]
    hits = []
    for _, image, box in detections:
        best, best_iou = -1, iou_thr
        for j, g in enumerate(gts[image]):
            value = iou(box, g)
            if not used[image][j] and value >= best_iou:
                best, best_iou = j, value
        if best >= 0:
            used[image][best] = True
        hits.append(best >= 0)
    if not hits:
        return 0.0
    tp = np.cumsum(hits)
    fp = np.cumsum(~np.array(hits))
    recall = tp / total
    precision = tp / (tp + fp)
    recall = np.concatenate([[0.0], recall, [1.0]])
    precision = np.concatenate([[0.0], precision, [0.0]])
    precision = np.maximum.accumulate(precision[::-1])[::-1]
    steps = np.flatnonzero(recall[1:] != recall[:-1])
    return float(np.sum((recall[steps + 1] - recall[steps]) * precision[steps + 1]))
