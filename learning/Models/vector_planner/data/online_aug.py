from collections import Counter

import numpy as np
import torch

from Models.common.projection import load_camera
from Models.common.vectormap import load_vector_map
from Models.vector_planner.data.dataset import OK_STATUS
from Models.vector_planner.data.frame_label import TRACE_HORIZON_M, build_frame_label, load_frame_meta, make_config
from Models.vector_planner.data.geo_aug import (MIN_BOX_SIDE_PX, MIN_BOX_VISIBLE, REF_HEIGHT, REF_WIDTH,
                                                synthesize_view, virtual_world_pose)
from Models.vector_planner.data.map_labels import (DEFAULT_PARAMS, densify, load_line_map, smooth_reference, to_base,
                                                   trace_slots)
from Models.vector_planner.data.paste_aug import (IMAGE_HEIGHT, IMAGE_WIDTH, S_RANGE, ground_point, load_sprite_bank,
                                                  paste_sprite, sample_pylon_groups, sample_pylons, sample_recolor)

MIN_DEPTH_M = 0.5
PERSON_S_RANGE = (1.5, 15.0)
PERSON_D_RANGE = (-1.5, 1.5)
PERSON_HEIGHT_RANGE = (1.5, 1.8)
PERSON_WIDTH = 0.6
HEIGHT_JITTER = 0.1
BRIGHTNESS_JITTER = 0.15


class _RootMeta(dict):
    # image_root ごとの (poses, gnss, obstacles) を最初の利用時に 1 回だけ読む
    def __missing__(self, image_root):
        self[image_root] = meta = load_frame_meta(image_root)
        return meta


def slots_consistent(original, new):
    return set(original.get('paths', {})) == set(new.get('paths', {}))


def shift_boxes(boxes, camera, dx, dy, dyaw, image_size):
    # 箱の下端中心を地面点とみなして仮想姿勢から見た位置・大きさに写す。見えなくなった箱は落とす
    height, width = image_size
    c, s = np.cos(dyaw), np.sin(dyaw)
    out = []
    for box in boxes:
        hit = ground_point(camera, box['cx'] * width, (box['cy'] + box['h'] / 2) * height)
        if hit is None:
            continue
        point, depth = hit
        rel = point - np.array([dx, dy])
        virtual = np.array([c * rel[0] + s * rel[1], -s * rel[0] + c * rel[1], 0.0])
        new_depth = float((virtual - camera.translation) @ camera.rotation[:, 2])
        if new_depth < MIN_DEPTH_M:
            continue
        uv, visible = camera.project(virtual[None])
        if not visible[0]:
            continue
        ratio = depth / new_depth
        w, h = box['w'] * width * ratio, box['h'] * height * ratio
        x0, x1 = uv[0, 0] - w / 2, uv[0, 0] + w / 2
        y0, y1 = uv[0, 1] - h, uv[0, 1]
        cx0, cx1 = max(x0, 0.0), min(x1, float(width))
        cy0, cy1 = max(y0, float(camera.pad_top)), min(y1, float(height - camera.pad_top))
        vw, vh = cx1 - cx0, cy1 - cy0
        if vw < MIN_BOX_SIDE_PX or vh < MIN_BOX_SIDE_PX or (vw * vh) / max(w * h, 1e-9) < MIN_BOX_VISIBLE:
            continue
        out.append({**box, 'cx': float((cx0 + cx1) / 2 / width), 'cy': float((cy0 + cy1) / 2 / height),
                    'w': float(vw / width), 'h': float(vh / height)})
    return out


class PoseShift:
    """記録姿勢を (0, dy, dyaw) ずらした仮想姿勢から、画像（地面ホモグラフィ）とラベルを作り直す。"""

    def __init__(self, osm_path, params_path=DEFAULT_PARAMS, p=0.5, dy_max=0.7, dyaw_max_deg=6.0,
                 teacher_lengths=None):
        self.osm_path, self.params_path = osm_path, params_path
        self.p, self.dy_max, self.dyaw_max_deg = p, dy_max, dyaw_max_deg
        self.teacher_lengths = teacher_lengths
        self.stats = Counter()
        self._loaded = False
        self._roots = _RootMeta()

    def _load(self):
        if self._loaded:
            return
        self.cfg = make_config(self.params_path, self.teacher_lengths)
        self.camera = load_camera(self.params_path, 'nHD', REF_HEIGHT, REF_WIDTH)
        self.line_map, self.vector_map = load_line_map(self.osm_path), load_vector_map(self.osm_path)
        self._loaded = True

    def sample(self):
        # 確率 p で (dy[m], dyaw[rad]) を引く。None は引かない。torch の乱数（worker ごとに seed 済み）
        if float(torch.rand(())) >= self.p:
            return None
        dy = float((torch.rand(()) * 2.0 - 1.0) * self.dy_max)
        dyaw = float(np.radians((float(torch.rand(())) * 2.0 - 1.0) * self.dyaw_max_deg))
        return dy, dyaw

    def apply(self, image_root, split, stem, label, image, dy, dyaw):
        # (image, label, 仮想姿勢 (x, y, yaw))。姿勢なし・ラベル再生成に失敗したら元のまま (image, label, None)
        poses, gnss, obstacles = self._roots[image_root]
        key = f'{split}/{stem}'
        pose = poses.get(key)
        if pose is None:
            self.stats['no_pose'] += 1
            return image, label, None
        self._load()
        vx, vy, vyaw = virtual_world_pose((pose['x'], pose['y'], pose['yaw']), dy, dyaw)
        existing = [tuple(o) for o in obstacles.get(gnss.get(key, {}).get('bag'), [])]
        new = build_frame_label(self.line_map, self.vector_map, (vx, vy), vyaw, existing, self.cfg)
        entry = new['paths'].get('straight')
        if entry is None or entry['status'] not in OK_STATUS or not slots_consistent(label, new):
            self.stats['fallback'] += 1
            return image, label, None
        height, width = image.shape[:2]
        new['boxes'] = shift_boxes(label.get('boxes', []), self.camera, 0.0, dy, dyaw, (height, width))
        new['has_boxes'] = bool(label.get('has_boxes', False))
        new['source'] = key
        self.stats['shifted'] += 1
        return synthesize_view(image, self.camera, 0.0, dy, dyaw), new, (vx, vy, vyaw)


class OnlinePaste:
    """straight が clear の元フレームへランダムなコーン（と人）を貼り、ラベルを作り直す。stop になる配置は引き直し、駄目なら元を返す。"""

    def __init__(self, osm_path, bank_path, params_path=DEFAULT_PARAMS, prob=0.7, pylon_height=0.7, pylon_width=0.25,
                 redraws=3, seed=None, teacher_lengths=None, group_prob=0.0,
                 group_sizes=(1, 2, 3, 4), group_spacing=(0.8, 1.0), hue_prob=0.0, far_degrade_prob=0.0,
                 s_range=None, person_prob=0.0, person_bank_path=None, person_s_range=None,
                 person_width=PERSON_WIDTH, person_height_range=PERSON_HEIGHT_RANGE, person_d_range=PERSON_D_RANGE):
        self.osm_path, self.bank_path, self.params_path = osm_path, bank_path, params_path
        self.prob, self.pylon_height, self.pylon_width, self.redraws = prob, pylon_height, pylon_width, redraws
        self.seed = seed
        self.teacher_lengths = teacher_lengths
        self.group_prob, self.group_sizes, self.group_spacing = group_prob, tuple(group_sizes), tuple(group_spacing)
        self.hue_prob, self.far_degrade_prob = hue_prob, far_degrade_prob
        self.s_range = tuple(S_RANGE if s_range is None else s_range)
        if person_prob > 0.0 and not person_bank_path:
            raise ValueError('person_prob > 0 には person_bank_path が必要')
        self.person_prob, self.person_bank_path, self.person_width = person_prob, person_bank_path, person_width
        self.person_s_range = tuple(float(v) for v in (PERSON_S_RANGE if person_s_range is None else person_s_range))
        self.person_height_range = tuple(person_height_range)
        self.person_d_range = tuple(person_d_range)
        self.stats = Counter()
        self._loaded = False
        self._roots = _RootMeta()
        self._rng, self._rng_key = None, None

    def _load(self):
        if self._loaded:
            return
        self.cfg = make_config(self.params_path, self.teacher_lengths)
        self.camera = load_camera(self.params_path, 'nHD', IMAGE_HEIGHT, IMAGE_WIDTH)
        self.bank = load_sprite_bank(self.bank_path)
        self.person_bank = load_sprite_bank(self.person_bank_path) if self.person_prob > 0.0 else None
        self.line_map, self.vector_map = load_line_map(self.osm_path), load_vector_map(self.osm_path)
        self._loaded = True

    def rng(self):
        key = self.seed if self.seed is not None else torch.initial_seed()
        if self._rng is None or (self.seed is None and key != self._rng_key):
            self._rng, self._rng_key = np.random.default_rng(int(key)), key
        return self._rng

    def _paste_all(self, image, reference, position, yaw, rng):
        # 遠い順に貼る。pasted は (x, y, base, 障害物幅)
        current, pasted, boxes = image, [], []
        if self.group_prob > 0.0 and rng.random() < self.group_prob:
            sampled = sample_pylon_groups(reference, rng, self.group_sizes, self.group_spacing,
                                          s_range=self.s_range)
        else:
            sampled = sample_pylons(reference, rng, s_range=self.s_range)
        items = [(s, d, x, y, False) for s, d, x, y in sampled]
        if self.person_prob > 0.0 and rng.random() < self.person_prob:
            items += [(s, d, x, y, True) for s, d, x, y in
                      sample_pylons(reference, rng, s_range=self.person_s_range, d_range=self.person_d_range)]
        colors = {}
        for s, d, x, y, is_person in sorted(items, key=lambda p: -p[0]):
            base = to_base(np.array([[x, y]]), position, yaw)[0]
            if is_person:
                sprite = self.person_bank[int(rng.integers(len(self.person_bank)))]
                result = paste_sprite(current, sprite, base, self.camera, float(rng.uniform(*self.person_height_range)),
                                      rng, gain_jitter=BRIGHTNESS_JITTER, far_degrade_prob=self.far_degrade_prob)
                if result is None:
                    continue
                result[1]['class'] = 1
                width = self.person_width
            else:
                sprite = self.bank[int(rng.integers(len(self.bank)))]
                height = self.pylon_height * rng.uniform(1.0 - HEIGHT_JITTER, 1.0 + HEIGHT_JITTER)
                recolor = None
                if self.hue_prob > 0.0:
                    if s not in colors:
                        colors[s] = sample_recolor(rng) if rng.random() < self.hue_prob else None
                    recolor = colors[s]
                result = paste_sprite(current, sprite, base, self.camera, height, rng, gain_jitter=BRIGHTNESS_JITTER,
                                      far_degrade_prob=self.far_degrade_prob, recolor=recolor)
                if result is None:
                    continue
                width = self.pylon_width
            current = result[0]
            boxes.append(result[1])
            pasted.append((x, y, base, width))
        return current, pasted, boxes

    def apply(self, image_root, split, stem, label, image, rng=None, pose_override=None):
        # pose_override=(x, y, yaw): 姿勢シフト後の仮想姿勢に貼るとき記録姿勢の代わりに使う
        straight = label.get('paths', {}).get('straight')
        rng = rng or self.rng()
        if straight is None or straight['status'] != 'clear' or rng.random() >= self.prob:
            self.stats['original'] += 1
            return image, label
        self._load()
        poses, gnss, obstacles = self._roots[image_root]
        key = f'{split}/{stem}'
        pose = poses.get(key) if pose_override is None else dict(zip(('x', 'y', 'yaw'), pose_override))
        traces, trace = None, None
        if pose is not None:
            position = np.array([pose['x'], pose['y']], float)
            traces = trace_slots(self.vector_map, position, pose['yaw'], horizon=TRACE_HORIZON_M)
            trace = traces['straight'][0] if 'straight' in traces else None
        if trace is None:
            self.stats['original'] += 1
            return image, label
        yaw = pose['yaw']
        reference = smooth_reference(densify(trace))
        existing = [tuple(o) for o in obstacles.get(gnss.get(key, {}).get('bag'), [])]
        cache = {}

        reason = 'fallback_no_visible'
        for attempt in range(1 + self.redraws):
            current, pasted, boxes = self._paste_all(image, reference, position, yaw, rng)
            if not pasted:
                self.stats['no_visible'] += 1
                reason = 'fallback_no_visible'
                continue
            new = build_frame_label(self.line_map, self.vector_map, position, yaw,
                                    existing + [(x, y, w) for x, y, _, w in pasted], self.cfg,
                                    corridors=False, cache=cache, map_elements=label['map_elements'],
                                    traces=traces)
            entry = new['paths'].get('straight')
            if entry is None or entry['status'] not in OK_STATUS:
                if attempt < self.redraws:
                    self.stats['redraw'] += 1
                reason = 'fallback_stop'
                continue
            for slot, path in new['paths'].items():
                corridor = label['paths'].get(slot, {}).get('corridor')
                if corridor is not None:
                    path['corridor'] = corridor
            new['boxes'] = list(label.get('boxes', [])) + boxes
            new['has_boxes'] = True
            new['pasted_base'] = [[float(b[0]), float(b[1]), w] for _, _, b, w in pasted]
            new['source'] = key
            self.stats[entry['status']] += 1
            return current, new
        self.stats[reason] += 1
        return image, label
