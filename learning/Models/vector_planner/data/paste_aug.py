import copy

import cv2
import numpy as np

IMAGE_HEIGHT, IMAGE_WIDTH = 384, 640
S_RANGE = (5.0, 18.0)
D_RANGE = (-1.2, 1.2)
MIN_PYLON_GAP_M = 0.8
BRIGHTNESS_STRENGTH = 0.5
HUE_PALETTE_DEG = (0.0, 15.0, 50.0, 120.0, 220.0)
HUE_SAT_THRESHOLD = 0.35
HUE_SAT_RANGE = (0.8, 1.1)
HUE_VAL_RANGE = (0.55, 1.05)
FAR_DEGRADE_MAX_H_PX = 24
FAR_DEGRADE_PAD_PX = 3
FAR_BLUR_SIGMA = (0.4, 1.0)
FAR_NOISE_MAX_STD = 12.0
FLIP_STATE = {'red': 'green', 'green': 'red'}
SAT_MIN, VAL_MIN = 90, 90
WHITEOUT_VAL, WHITEOUT_SAT_MAX, WHITEOUT_MAX_FRAC = 250, 60, 0.15


def ground_point(camera, u, v):
    ray = np.array([(u - camera.cx) / camera.fx, (v - camera.cy) / camera.fy, 1.0])
    direction = camera.rotation @ ray
    if direction[2] >= -1e-9:
        return None
    t = -camera.translation[2] / direction[2]
    if t <= 0:
        return None
    return (camera.translation + t * direction)[:2], float(t)


def _luminance(bgr):
    return bgr[..., 0] * 0.114 + bgr[..., 1] * 0.587 + bgr[..., 2] * 0.299


def _context_luminance(image, x0, y0, x1, y1, pad_top):
    height, width = image.shape[:2]
    bw, bh = x1 - x0, y1 - y0
    ex0, ex1 = max(0, x0 - bw // 2), min(width, x1 + bw // 2)
    ey0, ey1 = max(pad_top, y0 - bh // 2), min(height - pad_top, y1 + bh // 2)
    region = _luminance(image[ey0:ey1, ex0:ex1].astype(np.float32))
    keep = np.ones(region.shape, bool)
    keep[y0 - ey0:y1 - ey0, x0 - ex0:x1 - ex0] = False
    return float(region[keep].mean()) if keep.any() else float(region.mean())


def sample_recolor(rng, palette=HUE_PALETTE_DEG):
    # (目標色相 [deg], 彩度倍率, 明度倍率) を 1 組引く。列で共有する色として使う
    hue = float(palette[int(rng.integers(len(palette)))])
    return hue, float(rng.uniform(*HUE_SAT_RANGE)), float(rng.uniform(*HUE_VAL_RANGE))


def recolor_sprite(sprite_bgra, target_hue_deg, rng, sat_threshold=HUE_SAT_THRESHOLD, sat_scale=None, val_scale=None):
    # 彩度が sat_threshold 以上の画素（コーン本体）だけ平均色相が target_hue_deg になるよう回す。低彩度画素とアルファは不変
    if sat_scale is None:
        sat_scale = float(rng.uniform(*HUE_SAT_RANGE))
    if val_scale is None:
        val_scale = float(rng.uniform(*HUE_VAL_RANGE))
    out = sprite_bgra.copy()
    hsv = cv2.cvtColor(np.ascontiguousarray(sprite_bgra[..., :3]), cv2.COLOR_BGR2HSV).astype(np.float32)
    mask = (hsv[..., 1] / 255.0 >= sat_threshold) & (sprite_bgra[..., 3] > 0)
    if not mask.any():
        return out
    angle = hsv[..., 0][mask] * 2.0 * np.pi / 180.0
    mean = np.degrees(np.arctan2(np.sin(angle).mean(), np.cos(angle).mean())) % 360.0
    shift = (target_hue_deg - mean) / 2.0
    hsv[..., 0][mask] = (hsv[..., 0][mask] + shift) % 180.0
    hsv[..., 1][mask] = np.clip(hsv[..., 1][mask] * sat_scale, 0, 255)
    hsv[..., 2][mask] = np.clip(hsv[..., 2][mask] * val_scale, 0, 255)
    bgr = cv2.cvtColor(np.round(hsv).astype(np.uint8), cv2.COLOR_HSV2BGR)
    out[..., :3][mask] = bgr[mask]
    return out


def _far_degrade(out, before, x0, y0, w_i, h_i, alpha, pad_top, rng):
    # 貼った領域の周辺だけ、アルファで重み付けしたぼかしとノイズをかける（背景は変えない）
    height, width = out.shape[:2]
    rx0, rx1 = max(0, x0 - FAR_DEGRADE_PAD_PX), min(width, x0 + w_i + FAR_DEGRADE_PAD_PX)
    ry0, ry1 = max(pad_top, y0 - FAR_DEGRADE_PAD_PX), min(height - pad_top, y0 + h_i + FAR_DEGRADE_PAD_PX)
    weight = np.zeros((ry1 - ry0, rx1 - rx0), np.float32)
    weight[y0 - ry0:y0 - ry0 + h_i, x0 - rx0:x0 - rx0 + w_i] = alpha
    weight = np.clip(cv2.GaussianBlur(cv2.dilate(weight, np.ones((3, 3), np.uint8)), (3, 3), 0) * 1.5, 0, 1)
    sigma = float(rng.uniform(*FAR_BLUR_SIGMA))
    region = out[ry0:ry1, rx0:rx1].astype(np.float32)
    blurred = cv2.GaussianBlur(region, (0, 0), sigma)
    background = weight < 0.05
    ref = _luminance(before[ry0:ry1, rx0:rx1].astype(np.float32))
    std = float(ref[background].std()) if background.sum() >= 4 else 0.0
    std = min(std, FAR_NOISE_MAX_STD) * float(rng.uniform(0.5, 1.0))
    noise = rng.normal(0.0, std, region.shape).astype(np.float32) if std > 0 else 0.0
    mixed = region * (1 - weight[..., None]) + (blurred + noise) * weight[..., None]
    out[ry0:ry1, rx0:rx1] = np.clip(mixed + 0.5, 0, 255).astype(np.uint8)


def paste_sprite(image, sprite, base_xy, camera, height_m, rng, gain_jitter=0.05, hue_prob=0.0, far_degrade_prob=0.0,
                 recolor=None):
    # recolor=(色相, 彩度倍率, 明度倍率) は hue_prob より優先（列で同色にする用）。hue_prob / far_degrade_prob が 0 なら乱数消費も変わらない
    point = np.array([base_xy[0], base_xy[1], 0.0])
    depth = float(((point - camera.translation) @ camera.rotation)[2])
    if depth <= 0.3:
        return None
    pixels, visible = camera.project(point)
    if not visible[0]:
        return None
    u, v = pixels[0]

    sprite_h, sprite_w = sprite.shape[:2]
    h_px = camera.fy * height_m / depth
    h_i, w_i = int(round(h_px)), int(round(h_px * sprite_w / sprite_h))
    if h_i < 4 or w_i < 2:
        return None
    height, width = image.shape[:2]
    x0, y1 = int(round(u - w_i / 2)), int(round(v))
    y0 = y1 - h_i
    if x0 < 0 or x0 + w_i > width or y0 < camera.pad_top or y1 > height - camera.pad_top:
        return None

    if recolor is None and hue_prob > 0.0 and rng.random() < hue_prob:
        recolor = sample_recolor(rng)
    if recolor is not None:
        sprite = recolor_sprite(sprite, recolor[0], rng, sat_scale=recolor[1], val_scale=recolor[2])
    interpolation = cv2.INTER_AREA if h_i < sprite_h else cv2.INTER_LINEAR
    resized = cv2.resize(sprite, (w_i, h_i), interpolation=interpolation)
    if rng.random() < 0.5:
        resized = resized[:, ::-1]
    alpha = resized[..., 3].astype(np.float32) / 255.0
    if min(h_i, w_i) >= 7:
        alpha = cv2.GaussianBlur(alpha, (3, 3), 0)
    color = resized[..., :3].astype(np.float32)

    weight = alpha.sum()
    if weight > 1e-3:
        foreground = float((_luminance(color) * alpha).sum() / weight)
        context = _context_luminance(image, x0, y0, x0 + w_i, y1, camera.pad_top)
        gain = float(np.clip(context / max(foreground, 1.0), 0.6, 1.4))
        gain = (1.0 + BRIGHTNESS_STRENGTH * (gain - 1.0)) * rng.uniform(1.0 - gain_jitter, 1.0 + gain_jitter)
        color = np.clip(color * gain, 0, 255)

    out = image.copy()
    region = out[y0:y1, x0:x0 + w_i].astype(np.float32)
    out[y0:y1, x0:x0 + w_i] = np.clip(region * (1 - alpha[..., None]) + color * alpha[..., None] + 0.5, 0, 255).astype(np.uint8)
    if far_degrade_prob > 0.0 and h_i < FAR_DEGRADE_MAX_H_PX and rng.random() < far_degrade_prob:
        _far_degrade(out, image, x0, y0, w_i, h_i, alpha, camera.pad_top, rng)
    box = {'class': 0, 'score': 1.0, 'cx': (x0 + w_i / 2) / width, 'cy': (y0 + h_i / 2) / height,
           'w': w_i / width, 'h': h_i / height}
    return out, box


def load_sprite_bank(path):
    data = np.load(path)
    return [data[f'sprite_{i}'] for i in range(int(data['count']))]


def sample_pylons(reference, rng, s_range=S_RANGE, d_range=D_RANGE):
    reference = np.asarray(reference, float)
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(reference, axis=0), axis=1))])
    tangent = np.gradient(reference, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-12)
    pylons = []
    for _ in range(int(rng.integers(1, 3))):
        s, d = float(rng.uniform(*s_range)), float(rng.uniform(*d_range))
        if s > arc[-1]:
            continue
        p = np.array([np.interp(s, arc, reference[:, 0]), np.interp(s, arc, reference[:, 1])])
        t = np.array([np.interp(s, arc, tangent[:, 0]), np.interp(s, arc, tangent[:, 1])])
        x, y = p + d * np.array([-t[1], t[0]]) / max(np.linalg.norm(t), 1e-12)
        if any(np.hypot(x - q[2], y - q[3]) < MIN_PYLON_GAP_M for q in pylons):
            continue
        pylons.append((s, d, float(x), float(y)))
    return pylons


def sample_pylon_groups(reference, rng, group_sizes=(1, 2, 3, 4), spacing_range=(0.8, 1.0), s_range=S_RANGE,
                        d_range=D_RANGE, max_groups=1):
    # 進行方向に直交する横一列のコーン列を 1〜max_groups 個置く。列のうち 1 個の d が d_range に入る。(s, d, x, y) のリスト
    reference = np.asarray(reference, float)
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(reference, axis=0), axis=1))])
    tangent = np.gradient(reference, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-12)
    pylons = []
    for _ in range(int(rng.integers(1, max_groups + 1))):
        n = int(group_sizes[int(rng.integers(len(group_sizes)))])
        s = float(rng.uniform(*s_range))
        gaps = rng.uniform(*spacing_range, size=max(n - 1, 0))
        offsets = np.concatenate([[0.0], np.cumsum(gaps)])
        anchor = int(rng.integers(n))
        d0 = float(rng.uniform(*d_range))
        if s > arc[-1]:
            continue
        p = np.array([np.interp(s, arc, reference[:, 0]), np.interp(s, arc, reference[:, 1])])
        t = np.array([np.interp(s, arc, tangent[:, 0]), np.interp(s, arc, tangent[:, 1])])
        normal = np.array([-t[1], t[0]]) / max(np.linalg.norm(t), 1e-12)
        row = []
        for off in offsets:
            d = d0 + float(off - offsets[anchor])
            x, y = p + d * normal
            row.append((s, d, float(x), float(y)))
        if any(np.hypot(x - q[2], y - q[3]) < MIN_PYLON_GAP_M for _, _, x, y in row for q in pylons):
            continue
        pylons.extend(row)
    return pylons


def _box_px(box, shape):
    h, w = shape[:2]
    x0, x1 = (box['cx'] - box['w'] / 2) * w, (box['cx'] + box['w'] / 2) * w
    y0, y1 = (box['cy'] - box['h'] / 2) * h, (box['cy'] + box['h'] / 2) * h
    return (max(int(np.floor(x0)), 0), max(int(np.floor(y0)), 0), min(int(np.ceil(x1)), w), min(int(np.ceil(y1)), h))


def whiteout_fraction(crop_bgr):
    if crop_bgr.size == 0:
        return 0.0
    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    return float(((hsv[..., 2] >= WHITEOUT_VAL) & (hsv[..., 1] < WHITEOUT_SAT_MAX)).mean())


def swap_hue(image, box_px):
    # bbox 内の赤/緑画素の色相を反転（H' = 60 - H）。白飛び画素と bbox 外は不変
    x0, y0, x1, y1 = box_px
    out = image.copy()
    crop = out[y0:y1, x0:x1]
    if crop.size == 0:
        return out
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hue = hsv[..., 0].astype(np.int32)
    colored = (hsv[..., 1] > SAT_MIN) & (hsv[..., 2] > VAL_MIN) & ((hue < 10) | (hue > 165) | ((hue > 40) & (hue < 95)))
    hsv[..., 0] = np.where(colored, (60 - hue) % 180, hue).astype(np.uint8)
    out[y0:y1, x0:x1] = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    return out


def hue_swap_frame(image, label, whiteout_max=WHITEOUT_MAX_FRAC):
    # 全パネル box（class 2/3）の赤⇔緑を入れ替え、class と signal.state も反転。白飛びが多いフレームは変換しない。(画像, ラベル, 変換したか)
    panels = [b for b in label.get('boxes', []) if b['class'] in (2, 3)]
    if not panels:
        return image, label, False
    boxes_px = [_box_px(b, image.shape) for b in panels]
    for x0, y0, x1, y1 in boxes_px:
        if whiteout_fraction(image[y0:y1, x0:x1]) > whiteout_max:
            return image, label, False
    out_image = image
    for box_px in boxes_px:
        out_image = swap_hue(out_image, box_px)
    out_label = copy.deepcopy(label)
    for b in out_label['boxes']:
        if b['class'] in (2, 3):
            b['class'] = 5 - b['class']
    signal = out_label.get('signal')
    if signal and signal['state'] in FLIP_STATE:
        signal['state'] = FLIP_STATE[signal['state']]
    return out_image, out_label, True


def hue_swap(image_bgr, label, rng):
    # dataset から呼ぶ契約: (画像, ラベル) を返す。rng は未使用
    out_image, out_label, swapped = hue_swap_frame(image_bgr, label)
    return (out_image, out_label) if swapped else (image_bgr, label)
