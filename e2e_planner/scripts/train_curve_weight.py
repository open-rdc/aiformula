#!/usr/bin/env python3
"""train.py にカーブのきつさによる損失の重み付けを足したもの。

train.py はそのまま残してあるので、比較や切り戻しはそちらで行う。
設定は config/train_curve_weight.yaml (train.yaml に curve_weighting を足したもの)。
重み付けの中身は util/curve_weight.py を参照。
"""

import json
import sys
import time
import yaml
import torch
import torch.nn as nn
from torch.utils.data import ConcatDataset, Dataset, DataLoader, Subset, WeightedRandomSampler, random_split
from torch.utils.tensorboard import SummaryWriter
import cv2
import csv
from pathlib import Path
import numpy as np
from typing import List, Optional, Tuple
from tqdm import tqdm
from network import build_network
from util.preprocess import (
    IMAGE_WIDTH,
    IMAGE_HEIGHT,
    WAYPOINT_X_SCALE,
    WAYPOINT_Y_SCALE,
    WAYPOINT_Y_OFFSET,
    extract_red_mask,
    preprocess_mask,
    preprocess_rgb,
    normalize_waypoints,
    path_goes_backward,
)
from util.augment_train import horizontal_flip, jitter_stack, to_grayscale
from util.curve_weight import CurveWeighter, curve_score
from util.pylon_mask import is_pylon_near

NUM_WAYPOINTS = 20

# train/val の分割を run 間で固定する。拡張の有無で val が変わると比較できない
SPLIT_SEED = 42


def find_datasets(root: Path) -> List[Path]:
    """データセットのディレクトリを列挙する。

    root 直下に images/ があれば単一データセット。無ければ bag ごとに分けて
    作られた（create_data_from_bag.py の出力）とみなしてサブディレクトリを返す。
    """
    if (root / 'images').is_dir():
        return [root]

    subsets = sorted(d for d in root.iterdir() if d.is_dir() and (d / 'images').is_dir())
    if not subsets:
        raise FileNotFoundError(f'images/ を持つディレクトリが見つからない: {root}')
    return subsets


class E2EDataset(Dataset):
    """白線マスク画像と対応するRGB画像のペアを返す"""

    def __init__(self, dataset_path: Path, augment_config: dict = None,
                 frame_stack_distances: List[float] = None, use_pylon_mask: bool = False,
                 use_pylon_scenes: bool = False, drop_backward_paths: bool = False,
                 holdout_ranges: List[Tuple[int, int]] = (), holdout_gap: int = 0, avoid_only: bool = False,
                 blue_course_only: bool = False, keep_avoid_off_course: bool = False,
                 keep_only_ranges: List[Tuple[int, int]] = None, draw_pylon_boxes: bool = False,
                 past_mask_distances: List[float] = None, past_mask_mode: str = 'obs',
                 extra_rgb_dirs: List[Path] = ()):
        # augment_config が None のときは拡張なし（検証用）
        self.augment_config = augment_config
        # 過去フレームを何 m 後ろから引くか。時間ではなく距離で引く:
        # 同じ「1秒前」でも 2 m/s なら 2 m 後ろ、4 m/s なら 4 m 後ろになり、
        # 速度によって入力の意味が変わってしまう（waypoint を距離パラメータに
        # 変えたのと同じ理由）
        self.frame_stack_distances = list(frame_stack_distances or [])
        self.dataset_path = dataset_path
        self.mask_images_dir = dataset_path / 'mask_images'
        self.rgb_images_dir = dataset_path / 'images'
        self.path_dir = dataset_path / 'path'
        # パイロンの枠を塗ったマスク (create_pylon_masks.py)。白線マスクの 2 ch 目として重ねる
        self.use_pylon_mask = use_pylon_mask
        self.pylon_masks_dir = dataset_path / 'pylon_masks'
        # 障害物の枠を、マスクの ch ではなく RGB 画像に枠線で描き込む (draw_boxes_on_rgb)
        self.draw_pylon_boxes = draw_pylon_boxes
        # 過去のフレームの RGB ではなく、マスク (past_mask_mode) を何 m 前から ch として足すか
        self.past_mask_distances = list(past_mask_distances or [])
        if past_mask_mode not in PAST_MASK_VALUES:
            raise ValueError(f'past_mask_mode は {sorted(PAST_MASK_VALUES)} のどれか: {past_mask_mode}')
        self.past_mask_mode = past_mask_mode

        if not self.rgb_images_dir.exists():
            raise FileNotFoundError(f'RGB images directory not found: {self.rgb_images_dir}')
        if not self.mask_images_dir.exists():
            raise FileNotFoundError(
                f'mask images directory not found: {self.mask_images_dir}\n'
                '先に scripts/e2e_add_yolopv2_data.py でマスクを作ること')

        if (use_pylon_mask or draw_pylon_boxes or self.past_mask_distances) and not self.pylon_masks_dir.exists():
            raise FileNotFoundError(
                f'pylon masks directory not found: {self.pylon_masks_dir}\n'
                '先に scripts/create_pylon_masks.py でパイロンマスクを作ること')

        # 人が確認ツール (pylon_scene_web.py) で引いた区間。{stem: (避ける区間, 確認済み, 学習に使う)}
        scenes = self._read_pylon_scenes(dataset_path) if use_pylon_scenes else {}
        # 青コースの上を走っているか (label_blue_course.py が書く course.json)。{stem: 0/1}
        course = self._read_course(dataset_path) if blue_course_only else {}

        self.mask_files = [
            mask_file for mask_file in sorted(self.mask_images_dir.glob('*.png'))
            if (self.rgb_images_dir / mask_file.name).exists()
            and (self.path_dir / f'{mask_file.stem}.csv').exists()
            and (not (use_pylon_mask or draw_pylon_boxes) or (self.pylon_masks_dir / mask_file.name).exists())
            and scenes.get(mask_file.stem, (0, 0, 1))[2]
            # 青コースの外は外す。keep_avoid_off_course なら避ける区間 (確認済み) は青コースの外でも残す
            and (not blue_course_only or course.get(mask_file.stem, 0)
                 or (keep_avoid_off_course and scenes.get(mask_file.stem, (0, 0, 1))[0]
                     and scenes.get(mask_file.stem, (0, 0, 1))[1]))
            # avoid_only の bag (red コースなど) は「避ける区間 (確認済み)」のフレームだけ使う
            and (not avoid_only or (scenes.get(mask_file.stem, (0, 0, 1))[0] and scenes.get(mask_file.stem, (0, 0, 1))[1]))
            and not (drop_backward_paths
                     and path_goes_backward(np.array(self._read_waypoints(self.path_dir / f'{mask_file.stem}.csv'))))
            # 評価用の範囲と、その前後 holdout_gap フレーム (ほぼ同じ画像が学習に入らないように)
            and not any(a - holdout_gap <= int(mask_file.stem) <= b + holdout_gap for a, b in holdout_ranges)
            # 評価用の範囲だけを読む (Trainer が評価用の値を TensorBoard に出すため)
            and (keep_only_ranges is None or any(a <= int(mask_file.stem) <= b for a, b in keep_only_ranges))
        ]
        # データ拡張で作った RGB (Cosmos で天候を変えたもの・コースの外に物を貼ったもの)。
        # <dir>/<bag 名>/images/<番号>.png (か .jpg) があるフレームは、元の RGB とは別のサンプルとして足す。
        # 経路・白線マスク・障害物マスクは元のフレームのものを使う (拡張しても正解は変わらない)。
        # ただし <dir>/<bag 名>/pylon_masks/<番号>.png があれば、障害物マスクはそちらを使う
        # (パイロンを人に置き換えたフレームでは、人の枠を描いたマスクにする)
        self.rgb_files = [self.rgb_images_dir / f.name for f in self.mask_files]
        self.pylon_mask_files = [self.pylon_masks_dir / f.name for f in self.mask_files]
        # サンプルごとの拡張のディレクトリ (<dir>/<bag 名>)。元のサンプルは None。
        # 過去フレームの RGB・障害物マスクも、同じ拡張にあればそちらを使う (天候を変えた今のフレームに、
        # 元の晴れの過去フレームを重ねないように)。無いフレームは元のものを使う
        self.extra_bag_dirs = [None] * len(self.mask_files)
        # 拡張を足す元は、元のフレームだけ (前の拡張のディレクトリで足したサンプルに、次の拡張をまた足さない)
        base_files = list(self.mask_files)
        for extra in extra_rgb_dirs:
            extra_dir = Path(extra) / dataset_path.name / 'images'
            if not extra_dir.is_dir():
                continue
            for f in base_files:
                for cand in (extra_dir / f.name, extra_dir / f'{f.stem}.jpg'):
                    if cand.exists():
                        self.mask_files.append(f)
                        self.rgb_files.append(cand)
                        override = Path(extra) / dataset_path.name / 'pylon_masks' / f.name
                        self.pylon_mask_files.append(override if override.exists() else self.pylon_masks_dir / f.name)
                        self.extra_bag_dirs.append(Path(extra) / dataset_path.name)
                        break

        # 学習で多めに引くサンプル。use_pylon_scenes なら人が「避ける区間」と確認したフレーム
        # (未確認の印は数えない)、そうでなければコーンが近いフレーム
        if use_pylon_scenes:
            self.pylon_near = np.array([
                bool(scenes.get(f.stem, (0, 0, 1))[0] and scenes.get(f.stem, (0, 0, 1))[1])
                for f in self.mask_files], dtype=bool)
        else:
            self.pylon_near = np.array([
                use_pylon_mask and is_pylon_near(self._read_pylon_mask(f.name)) for f in self.mask_files], dtype=bool)

        # 重みは train 全体の分布から決めるので、Trainer が後から CurveWeighter を
        # 差し込む。val は None のまま (= 重み 1)
        self.curve_weighter = None
        # 避ける区間 (pylon_near) の損失に掛ける倍率。Trainer が train のときだけ設定する
        self.avoid_loss_weight = 1.0
        self.curve_scores = np.array([
            curve_score(self._read_waypoints(self.path_dir / f'{f.stem}.csv'))
            for f in self.mask_files], dtype=np.float32)

        self.arc_by_stem = {}
        self.stems = [mask_file.stem for mask_file in self.mask_files]
        if self.frame_stack_distances or self.past_mask_distances:
            self._load_arc_lengths()

    @staticmethod
    def _read_course(dataset_path: Path) -> dict:
        f = dataset_path / 'course.json'
        if not f.exists():
            raise FileNotFoundError(
                f'course.json が無い: {dataset_path}\n'
                '先に scripts/label_blue_course.py で青コースの上を走っているかを判定すること')
        j = json.loads(f.read_text())
        return dict(zip(j['stems'], j['blue']))

    @staticmethod
    def _read_pylon_scenes(dataset_path: Path) -> dict:
        f = dataset_path / 'pylon_scenes.json'
        if not f.exists():
            return {}
        j = json.loads(f.read_text())
        use = j.get('use', [1] * len(j['stems']))
        return {s: (sc, rv, u) for s, sc, rv, u in zip(j['stems'], j['scene'], j['reviewed'], use)}

    def _past_rgb_file(self, stem: str, extra: Optional[Path]) -> Path:
        for cand in ([extra / 'images' / f'{stem}.jpg', extra / 'images' / f'{stem}.png'] if extra else []):
            if cand.exists():
                return cand
        return self.rgb_images_dir / f'{stem}.png'

    def _past_pylon_mask_file(self, stem: str, extra: Optional[Path]) -> Path:
        if extra and (extra / 'pylon_masks' / f'{stem}.png').exists():
            return extra / 'pylon_masks' / f'{stem}.png'
        return self.pylon_masks_dir / f'{stem}.png'

    def _past_mask(self, stem: str, extra: Optional[Path] = None) -> np.ndarray:
        """過去のフレームのマスク (float32, 0〜1)。白線と障害物を past_mask_mode の値で 1 枚に描く"""
        lane_value, obs_value = PAST_MASK_VALUES[self.past_mask_mode]
        out = np.zeros((360, 640), np.float32)
        if lane_value:
            lane = extract_red_mask(cv2.imread(str(self.mask_images_dir / f'{stem}.png'), cv2.IMREAD_COLOR))
            out[lane > 0] = lane_value
        obs = self._read_pylon_mask_file(self._past_pylon_mask_file(stem, extra)) > 127
        if obs.shape != out.shape:
            out = cv2.resize(out, (obs.shape[1], obs.shape[0]), interpolation=cv2.INTER_NEAREST)
        out[obs] = obs_value  # 重なる所は障害物を優先
        return out

    def _read_pylon_mask(self, name: str) -> np.ndarray:
        return self._read_pylon_mask_file(self.pylon_masks_dir / name)

    @staticmethod
    def _read_pylon_mask_file(path: Path) -> np.ndarray:
        mask = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if mask is None:
            raise RuntimeError(f'Failed to load pylon mask: {path.name}')
        return mask

    @staticmethod
    def _read_waypoints(csv_file: Path) -> List[List[float]]:
        with open(csv_file, 'r') as f:
            return [[float(row['x']), float(row['y'])] for row in csv.DictReader(f)]

    def _load_arc_lengths(self) -> None:
        """meta.csv から各サンプルの累積走行距離[m]を読む"""
        meta_file = self.dataset_path / 'meta.csv'
        if not meta_file.exists():
            raise FileNotFoundError(
                f'meta.csv が無い: {meta_file}\n'
                'create_data_from_bag.py --meta-only で作ること')

        with open(meta_file, 'r') as f:
            for row in csv.DictReader(f):
                self.arc_by_stem[f'{int(row["index"]):05d}'] = float(row['arc_m'])

        missing = [stem for stem in self.stems if stem not in self.arc_by_stem]
        if missing:
            raise RuntimeError(f'{self.dataset_path}: meta.csv に {len(missing)} 件ぶんの'
                               f'距離が無い（例 {missing[:3]}）')

        # 過去フレームは入力の画像なので、学習に使わないフレーム (青コースの外・評価用の前後・
        # 使わない範囲など) も含めた bag の全フレームから引く。学習に使うフレームだけから引くと、
        # 外したフレームの近くで本来より遠い画像を代わりに使ってしまう
        self.all_stems = sorted(stem for stem in self.arc_by_stem
                                if (self.rgb_images_dir / f'{stem}.png').exists())
        # index 順 = 収集順 = 距離の昇順。二分探索で過去フレームを引く
        self.arcs = np.array([self.arc_by_stem[stem] for stem in self.all_stems])
        self.all_pos = {stem: i for i, stem in enumerate(self.all_stems)}

    def _past_stems(self, idx: int, distances: List[float] = None) -> List[str]:
        """idx のサンプルから distances[m] (既定は frame_stack_distances) だけ後ろのサンプル番号。

        走り始めでその距離ぶん戻れない場合は最古のフレームで代用する
        （毎 bag の先頭数サンプルだけなので、現在フレームの複製で埋めるより自然）
        """
        distances = self.frame_stack_distances if distances is None else distances
        if not distances:
            return []

        pos = self.all_pos[self.stems[idx]]
        target = self.arcs[pos] - np.array(distances)
        # arcs は昇順。target を挟む 2 点のうち**近いほう**を取る。
        # 手前側だけを見ると、サンプル間隔 (2 m/s で約 0.5 m, 4 m/s で約 0.95 m) の
        # ぶんだけ常に遠い側へ偏る
        after = np.searchsorted(self.arcs, target, side='left')
        before = np.clip(after - 1, 0, len(self.arcs) - 1)
        after = np.clip(after, 0, len(self.arcs) - 1)
        nearer = np.where(np.abs(self.arcs[before] - target) <= np.abs(self.arcs[after] - target),
                          before, after)
        return [self.all_stems[i] for i in np.clip(nearer, 0, pos)]

    def __len__(self) -> int:
        return len(self.mask_files)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, ...]:
        mask_file = self.mask_files[idx]
        rgb_file = self.rgb_files[idx]
        csv_file = self.path_dir / f'{mask_file.stem}.csv'

        mask_bgr = cv2.imread(str(mask_file), cv2.IMREAD_COLOR)
        rgb_bgr = cv2.imread(str(rgb_file), cv2.IMREAD_COLOR)
        if mask_bgr is None or rgb_bgr is None:
            raise RuntimeError(f'Failed to load image pair: {mask_file.name}')

        # 新しい順に [現在, -d1 m, -d2 m, ...]
        rgb_frames = [rgb_bgr]
        for past_stem in self._past_stems(idx):
            past_file = self._past_rgb_file(past_stem, self.extra_bag_dirs[idx])
            past_bgr = cv2.imread(str(past_file), cv2.IMREAD_COLOR)
            if past_bgr is None:
                raise RuntimeError(f'Failed to load past frame: {past_file.name}')
            rgb_frames.append(past_bgr)

        waypoints = self._read_waypoints(csv_file)

        mask_binary = extract_red_mask(mask_bgr)
        pylon_binary = ((self._read_pylon_mask_file(self.pylon_mask_files[idx]) > 127).astype(np.uint8)
                        if self.use_pylon_mask or self.draw_pylon_boxes else None)
        # 過去フレームにも、そのフレームの障害物の枠を描く
        box_masks = ([pylon_binary] + [(self._read_pylon_mask_file(
            self._past_pylon_mask_file(st, self.extra_bag_dirs[idx])) > 127).astype(np.uint8)
                                       for st in self._past_stems(idx)]
                     if self.draw_pylon_boxes else [])
        waypoints_m = np.array(waypoints, dtype=np.float32)

        flipped = False
        if self.augment_config is not None:
            # numpy のグローバル RNG は DataLoader の worker 間で再シードされないため、
            # 全 worker が同じ乱数列を出してしまう。torch の RNG は worker ごとに
            # 正しくずらされるので、そこから種を引いて per-sample の Generator を作る
            rng = np.random.default_rng(int(torch.randint(0, 2**31 - 1, (1,)).item()))
            cfg = self.augment_config

            # 拡張はスタック内の全フレームに同じものをかける。
            # フレームごとに変えると、現実には起こらない見た目の差が生まれ、
            # モデルがフレーム間の差分を手がかりにできなくなる
            if rng.random() < cfg['flip_prob']:
                flipped = True
                flipped = [horizontal_flip(frame, mask_binary, waypoints_m)
                           for frame in rgb_frames]
                rgb_frames = [f[0] for f in flipped]
                mask_binary, waypoints_m = flipped[0][1], flipped[0][2]
                if pylon_binary is not None:
                    pylon_binary = np.ascontiguousarray(pylon_binary[:, ::-1])
                box_masks = [np.ascontiguousarray(m[:, ::-1]) for m in box_masks]
                if box_masks:
                    box_masks[0] = pylon_binary

            if rng.random() < cfg['color_prob']:
                rgb_frames = jitter_stack(
                    rgb_frames, int(rng.integers(0, 2**31 - 1)),
                    brightness=cfg['brightness'],
                    contrast=cfg['contrast'],
                    saturation=cfg['saturation'],
                    hue=cfg['hue'],
                )

            # 色そのものを落とす。色を手がかりにできない状態でも走れるようにする
            if rng.random() < cfg['gray_prob']:
                rgb_frames = [to_grayscale(frame) for frame in rgb_frames]

        # 枠は色の拡張 (色相・グレースケール化) の後に描く。前に描くと枠の色が変わる
        if self.draw_pylon_boxes:
            rgb_frames = [draw_boxes_on_rgb(frame, m) for frame, m in zip(rgb_frames, box_masks)]

        mask_channels = [preprocess_mask(mask_binary)]
        if self.use_pylon_mask:
            mask_channels.append(preprocess_mask(pylon_binary))
        # 過去のマスク。左右反転したときは過去のマスクも反転する
        for past_stem in self._past_stems(idx, self.past_mask_distances):
            past = self._past_mask(past_stem, self.extra_bag_dirs[idx])
            if flipped:
                past = np.ascontiguousarray(past[:, ::-1])
            mask_channels.append(preprocess_mask(past))
        mask_tensor = torch.from_numpy(np.concatenate(mask_channels, axis=0))
        # (3*(1+past), H, W) にチャネル方向で連結
        rgb_tensor = torch.from_numpy(
            np.concatenate([preprocess_rgb(frame) for frame in rgb_frames], axis=0))
        waypoints_tensor = torch.from_numpy(normalize_waypoints(waypoints_m))

        # 曲がり量は拡張の後で測る。左右反転をかけたら曲がる向きも入れ替わるため
        score = curve_score(waypoints_m)
        weight = 1.0 if self.curve_weighter is None else self.curve_weighter(score)
        # 避ける区間は損失も重くする (train だけ。val は curve_weighter が None なので 1 のまま)
        if self.curve_weighter is not None and self.pylon_near[idx]:
            weight *= self.avoid_loss_weight

        return (mask_tensor, rgb_tensor, waypoints_tensor,
                torch.tensor(weight, dtype=torch.float32), torch.tensor(score, dtype=torch.float32))

# 過去のマスクの描き方 {名前: (白線の値, 障害物の値)}。0 は描かない
PAST_MASK_VALUES = {
    'obs': (0.0, 1.0),         # 障害物だけ
    'merged': (0.5, 1.0),      # 白線 = 灰, 障害物 = 白
    'merged_inv': (1.0, 0.5),  # 白線 = 白, 障害物 = 灰
    'same': (1.0, 1.0),        # どちらも白
}

BOX_COLOR = (255, 0, 255)  # BGR のマゼンタ。コースにほぼ無い色
BOX_THICKNESS = 3          # 640x360 で 3 px (256x144 に縮めると約 1 px)


def draw_boxes_on_rgb(bgr: np.ndarray, box_mask: np.ndarray) -> np.ndarray:
    """障害物マスク (枠を塗りつぶしたもの) の輪郭を BGR 画像に枠線で描く。
    重なった枠は 1 つの輪郭になる。画像とマスクの大きさが違えばマスクを合わせる"""
    if box_mask is None or not box_mask.any():
        return bgr
    if box_mask.shape[:2] != bgr.shape[:2]:
        box_mask = cv2.resize(box_mask, (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST)
    contours, _ = cv2.findContours(box_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = np.ascontiguousarray(bgr.copy())
    cv2.drawContours(out, contours, -1, BOX_COLOR, BOX_THICKNESS)
    return out


def oversample_weights(pylon_near: np.ndarray, factor: float) -> np.ndarray:
    """コーンが近いサンプルを factor 倍引くための WeightedRandomSampler の重み"""
    return np.where(pylon_near, factor, 1.0)


class Config:
    def __init__(self, config_path: Path, package_root: Path, run_name: str):
        self.package_root = package_root
        self.run_name = run_name

        with open(config_path, 'r') as f:
            config_dict = yaml.safe_load(f)

        self.epochs = config_dict['epochs']
        self.batch_size = config_dict['batch_size']
        self.learning_rate = config_dict['learning_rate']
        self.num_workers = config_dict['num_workers']
        # 重みも run 名で分ける。run のたびに e2e_model.pt が上書きされて
        # 過去の学習結果が失われる事故を防ぐ
        base = Path(config_dict['weight_file'])
        self.weight_file = f'{base.stem}_{run_name}{base.suffix}'

        self.embed_dim = config_dict.get('embed_dim', 128)
        self.num_heads = config_dict.get('num_heads', 4)
        self.num_layers = config_dict.get('num_layers', 4)
        self.dropout = config_dict.get('dropout', 0.1)
        self.grad_clip_norm = config_dict.get('grad_clip_norm', 1.0)

        # 学習率スケジュール。'cosine' か 'none'
        self.lr_schedule = config_dict.get('lr_schedule', 'none')
        self.lr_min = float(config_dict.get('lr_min', 0.0))

        # マスクとRGBの融合方法。
        # 'late'  : モダリティ別の ConvStem -> Transformer で融合（従来）
        # 'early' : 画素のまま 4ch に重ねて ConvStem 1 本（トークン数が半分になる）
        self.fusion = config_dict.get('fusion', 'late')

        # 過去フレームを何 m 後ろから重ねるか。空なら現在フレームのみ（従来）。
        # 距離で引くので、収集速度が違う bag でも同じ幾何関係のスタックになる
        self.frame_stack_distances = config_dict.get('frame_stack_distances', []) or []
        if self.frame_stack_distances and self.fusion != 'early':
            raise ValueError('frame_stack_distances は fusion: early のときだけ使える')

        # データ拡張（学習側のみ。検証には適用しない）
        self.augment_config = {
            # 左右反転は既定で無効。コースは反時計回りで左折が右折の 4.86 倍あり、
            # 反転すると分岐で曲がる向きが競合する
            'flip_prob': config_dict.get('augment_flip_prob', 0.0),
            'color_prob': config_dict.get('augment_color_prob', 0.8),
            'brightness': config_dict.get('augment_brightness', 0.3),
            'contrast': config_dict.get('augment_contrast', 0.3),
            'saturation': config_dict.get('augment_saturation', 0.3),
            'hue': config_dict.get('augment_hue', 0.03),
            'gray_prob': config_dict.get('augment_gray_prob', 0.3),
        }
        self.augment_enabled = config_dict.get('augment', True)

        # パイロンマスクを白線マスクの 2 ch 目として入れるか。false なら従来どおり 1 ch
        self.pylon_mask = bool(config_dict.get('pylon_mask', False))
        # コーンが近いサンプルを何倍引くか。1.0 なら従来どおり一様に shuffle
        self.pylon_oversample = float(config_dict.get('pylon_oversample', 1.0))
        # true なら多めに引くのを、確認ツールで人が引いた「避ける区間」(pylon_scenes.json) にする。
        # 「学習に使わない」範囲も外す。false なら is_pylon_near (コーンが近いフレーム)
        self.pylon_scenes = bool(config_dict.get('pylon_scenes', False))
        # true なら正解の経路が後ろへ戻るサンプル (バックしている場面) を外す
        self.drop_backward_paths = bool(config_dict.get('drop_backward_paths', False))
        # 評価用に抜く範囲 {bag 名: [(始め, 終わり), ...]} (images/ の番号)。ファイルは設定ファイルからの相対パス
        self.holdout = {}
        if config_dict.get('holdout_file'):
            with open(Path(config_path).parent / config_dict['holdout_file'], 'r') as f:
                self.holdout = {name: [tuple(r) for r in ranges] for name, ranges in (yaml.safe_load(f) or {}).items()}
        self.holdout_gap = int(config_dict.get('holdout_gap', 0))
        # 避ける区間 (pylon_scenes.json で確認済み) のフレームだけを使う bag。コースが違う red など
        self.avoid_only_datasets = list(config_dict.get('avoid_only_datasets', []) or [])
        # true なら青コースの上を青コースの向きに走っているフレームだけ使う (course.json)
        self.blue_course_only = bool(config_dict.get('blue_course_only', False))
        # true なら避ける区間 (確認済み) は青コースの外でも使う (本人, 2026-10-31)
        self.keep_avoid_off_course = bool(config_dict.get('keep_avoid_off_course', False))
        # 評価用 (学習にも best epoch の選択にも使わない) の bag。避ける区間だけを TensorBoard に出す
        self.test_datasets = list(config_dict.get('test_datasets', []) or [])
        # 避ける区間 (多めに引くサンプル) の損失を何倍にするか。1.0 なら重み付けしない
        self.avoid_loss_weight = float(config_dict.get('avoid_loss_weight', 1.0))
        # true なら障害物の枠を RGB 画像に枠線で描き込む (pylon_mask: false と組み合わせて、マスクは白線の 1 ch)
        self.pylon_bbox_on_rgb = bool(config_dict.get('pylon_bbox_on_rgb', False))
        # 過去のフレームのマスクを何 m 前から ch として足すか (過去の RGB は frame_stack_distances)
        self.past_mask_distances = list(config_dict.get('past_mask_distances', []) or [])
        self.past_mask_mode = str(config_dict.get('past_mask_mode', 'obs'))
        # データ拡張の RGB を置いたディレクトリ (<dir>/<bag 名>/images/)。train の bag にだけ足す
        self.extra_rgb_dirs = [Path(p).expanduser() for p in (config_dict.get('extra_rgb_dirs', []) or [])]
        if self.past_mask_distances and self.fusion != 'early':
            raise ValueError('past_mask_distances は fusion: early のときだけ使える')

        # カーブのきつさによる損失の重み付け (util/curve_weight.py)
        self.curve_weighting = config_dict.get('curve_weighting', {}) or {'mode': 'none'}
        # val でこれより |曲がり量| が大きいサンプルを「急カーブ」として別に集計する
        self.eval_tight_m = float(self.curve_weighting.get('eval_tight_m', 4.0))

        # train/val の分け方。'temporal' は時系列で後半を val にする（既定）。
        # 'random' は従来どおりランダム分割だが、隣接フレームのリークがある。
        # 'bag' は bag（サブデータセット）単位で val を切り分ける。bag をまたいで
        # 連結したデータでは index 順 = 時系列順にならないので temporal が使えない
        self.split_mode = config_dict.get('split_mode', 'temporal')
        self.split_gap = config_dict.get('split_gap', 25)
        # split_mode: 'bag' のとき val に回すサブディレクトリ名
        self.val_datasets = config_dict.get('val_datasets', []) or []
        # train からも val からも外すサブディレクトリ名（ラベルが信用できない bag）
        self.exclude_datasets = config_dict.get('exclude_datasets', []) or []

        self.weights_dir = package_root / 'weights'
        self.weights_dir.mkdir(exist_ok=True)

        # run ごとにサブディレクトリを切る。runs/ 直下に直接書くと
        # TensorBoard が全 run を "." という 1 つの run に合流させてしまう
        self.logs_dir = package_root / 'runs' / run_name

        self.device = torch.device('cuda')

class Trainer:
    def __init__(self, dataset_path: Path, config: Config):
        self.config = config
        self.device = config.device

        # 拡張ありの Subset は元の Dataset を共有するので、そのままだと
        # 検証にも拡張がかかる。拡張あり/なしの 2 インスタンスを作り、
        # 同じ index で Subset を組み直す
        aug_cfg = config.augment_config if config.augment_enabled else None
        subsets = find_datasets(dataset_path)

        if config.split_mode == 'bag':
            train_dataset, val_dataset = self._split_by_bag(subsets, config, aug_cfg)
        else:
            if len(subsets) > 1:
                raise ValueError(
                    f"split_mode: '{config.split_mode}' は単一データセット用。"
                    f'{len(subsets)} 個の bag が見つかったので split_mode: bag を使うこと')
            train_dataset, val_dataset = self._split_within_dataset(subsets[0], config, aug_cfg)

        self.subsets = subsets

        # 重みは train の分布だけから決める。val は重み 1 のままにして、
        # val loss を train.py の run と同じ尺度で比べられるようにする
        train_sources, train_scores = self._train_sources(train_dataset)
        self.curve_weighter = CurveWeighter(config.curve_weighting, train_scores)
        for source in train_sources:
            source.curve_weighter = self.curve_weighter
            source.avoid_loss_weight = config.avoid_loss_weight
        if config.avoid_loss_weight != 1.0:
            print(f'Avoid loss weight: 避ける区間の損失を {config.avoid_loss_weight} 倍')

        # コーンが近いサンプルを多めに引く。1 epoch のサンプル数は train 全体と同じ
        sampler = None
        if config.pylon_oversample > 1.0:
            train_near = self._train_pylon_near(train_dataset)
            sampler = WeightedRandomSampler(
                oversample_weights(train_near, config.pylon_oversample).tolist(),
                num_samples=len(train_dataset), replacement=True)
            print(f'Pylon {"scenes" if config.pylon_scenes else "near"}: {int(train_near.sum())} / {len(train_near)} 件を {config.pylon_oversample} 倍引く')

        self.train_loader = DataLoader(
            train_dataset,
            batch_size=config.batch_size,
            shuffle=sampler is None,
            sampler=sampler,
            num_workers=config.num_workers
        )
        self.val_loader = DataLoader(
            val_dataset,
            batch_size=config.batch_size,
            shuffle=False,
            num_workers=config.num_workers
        )
        # 避ける区間だけで測る組 (TensorBoard に出すだけで、best epoch の選択には使わない)
        self.avoid_loaders = self._avoid_loaders(subsets, config) if config.pylon_scenes else {}

        self.model = build_network(
            config.fusion,
            num_waypoints=NUM_WAYPOINTS,
            image_height=IMAGE_HEIGHT,
            image_width=IMAGE_WIDTH,
            embed_dim=config.embed_dim,
            num_heads=config.num_heads,
            num_layers=config.num_layers,
            dropout=config.dropout,
            **({'num_past_frames': len(config.frame_stack_distances),
                'num_mask_channels': (2 if config.pylon_mask else 1) + len(config.past_mask_distances)}
               if config.fusion == 'early' else {}),
        ).to(self.device)
        self.optimizer = torch.optim.AdamW(self.model.parameters(), lr=config.learning_rate)

        # T_max は全エポック数。途中で止めると学習率が下がりきらないので、
        # epochs を変えたら run をやり直すこと
        if config.lr_schedule == 'cosine':
            self.scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
                self.optimizer, T_max=config.epochs, eta_min=config.lr_min)
        elif config.lr_schedule == 'none':
            self.scheduler = None
        else:
            raise ValueError(f"lr_schedule は 'cosine' か 'none': {config.lr_schedule}")

        self.mseloss = nn.MSELoss()
        self.writer = SummaryWriter(log_dir=str(config.logs_dir))

        self.best_val_loss = float('inf')

        # どの run が何の設定だったか後から TensorBoard 上で追えるようにする
        self.writer.add_text('config', '\n'.join([
            f'dataset: {dataset_path}',
            f'bags: {len(subsets)}',
            f'train/val: {len(train_dataset)} / {len(val_dataset)}',
            f'split_mode: {config.split_mode}',
            f'val_datasets: {config.val_datasets}',
            f'exclude_datasets: {config.exclude_datasets}',
            f'augment: {aug_cfg}',
            f'epochs / batch / lr: {config.epochs} / {config.batch_size} / {config.learning_rate}',
            f'lr_schedule: {config.lr_schedule} (min {config.lr_min})',
            f'waypoint norm: x {WAYPOINT_X_SCALE} / y {WAYPOINT_Y_SCALE}',
            f'fusion: {config.fusion}',
            f'frame_stack_distances: {config.frame_stack_distances}',
            f'curve_weighting: {config.curve_weighting}',
            *self.curve_weighter.describe(),
        ]), 0)

        print(f'Using device: {self.device}')
        print(f'Datasets: {len(self.subsets)} ({dataset_path})')
        print(f'Train size: {len(train_dataset)}, Val size: {len(val_dataset)}')
        print(f'Augment (train only): {aug_cfg if aug_cfg else "disabled"}')
        print(f'Fusion: {config.fusion} '
              f'({sum(p.numel() for p in self.model.parameters()) / 1e6:.2f} M params)')
        print(f'Frame stack: ' + (f'現在 + 過去 {config.frame_stack_distances} m'
                                  if config.frame_stack_distances else '現在フレームのみ'))
        print(f'LR schedule: {config.lr_schedule}' +
              (f' ({config.learning_rate} -> {config.lr_min}, T_max={config.epochs})'
               if config.lr_schedule == 'cosine' else f' ({config.learning_rate} 固定)'))
        print(f'Split: {config.split_mode}' +
              (f' (gap={config.split_gap} samples)' if config.split_mode == 'temporal' else ''))
        print('\n'.join(self.curve_weighter.describe()))

    @staticmethod
    def _train_sources(train_dataset) -> Tuple[List['E2EDataset'], np.ndarray]:
        """train 側の E2EDataset と、train に入るサンプルの曲がり量"""
        if isinstance(train_dataset, ConcatDataset):
            sources = list(train_dataset.datasets)
            return sources, np.concatenate([d.curve_scores for d in sources])
        # Subset (temporal / random split)。同じ source の val 側とは別インスタンス
        return [train_dataset.dataset], train_dataset.dataset.curve_scores[train_dataset.indices]

    @staticmethod
    def _train_pylon_near(train_dataset) -> np.ndarray:
        """train に入るサンプルの並び順で、コーンが近いかどうか"""
        if isinstance(train_dataset, ConcatDataset):
            return np.concatenate([d.pylon_near for d in train_dataset.datasets])
        return train_dataset.dataset.pylon_near[train_dataset.indices]

    @staticmethod
    def weighted_mse(outputs: torch.Tensor, targets: torch.Tensor, weights: torch.Tensor) -> torch.Tensor:
        """重みが全部 1 なら nn.MSELoss と一致する"""
        return (((outputs - targets) ** 2).mean(dim=1) * weights).mean()

    @staticmethod
    def _avoid_loaders(subsets, config) -> dict:
        """{名前: DataLoader}。どれも避ける区間 (確認済み) のフレームだけ。
        ValAvoid = val の bag、Test/holdout = holdout_file で抜いた範囲、Test/testbag = test_datasets"""
        by_name = {d.name: d for d in subsets}

        def make(dirs_ranges):
            parts = []
            for d, ranges in dirs_ranges:
                if not (d / 'pylon_scenes.json').exists():
                    continue
                ds = E2EDataset(d, augment_config=None, frame_stack_distances=config.frame_stack_distances,
                                use_pylon_mask=config.pylon_mask, draw_pylon_boxes=config.pylon_bbox_on_rgb,
                        past_mask_distances=config.past_mask_distances, past_mask_mode=config.past_mask_mode, use_pylon_scenes=True, avoid_only=True,
                                drop_backward_paths=config.drop_backward_paths, keep_only_ranges=ranges)
                if len(ds):
                    parts.append(ds)
            return ConcatDataset(parts) if parts else None

        sets = {
            'ValAvoid': make([(by_name[n], None) for n in config.val_datasets if n in by_name]),
            'Test/holdout': make([(by_name[n], r) for n, r in config.holdout.items() if n in by_name]),
            'Test/testbag': make([(by_name[n], None) for n in config.test_datasets if n in by_name]),
        }
        loaders = {}
        for name, ds in sets.items():
            if ds is not None:
                print(f'{name}: 避ける区間 {len(ds)} 件 (TensorBoard に出すだけ)')
                loaders[name] = DataLoader(ds, batch_size=config.batch_size, shuffle=False,
                                           num_workers=config.num_workers)
        return loaders

    @staticmethod
    def _split_by_bag(subsets, config, aug_cfg):
        """bag 単位で train/val を分ける。

        bag をまたいで連結すると index 順が時系列順でなくなるので temporal split は使えない。
        val に回す bag を丸ごと外せば隣接フレームのリークは原理的に起きない。
        """
        names = {d.name for d in subsets}
        unknown = [name for name in config.val_datasets + config.exclude_datasets
                   if name not in names]
        if unknown:
            raise ValueError(f'val_datasets / exclude_datasets に存在しない名前がある: {unknown}\n'
                             f'候補: {sorted(names)}')

        overlap = set(config.val_datasets) & set(config.exclude_datasets)
        if overlap:
            raise ValueError(f'val_datasets と exclude_datasets が重複している: {sorted(overlap)}')

        if config.exclude_datasets:
            print('Excluded : ' + ', '.join(sorted(config.exclude_datasets)))
        subsets = [d for d in subsets if d.name not in config.exclude_datasets]

        val_dirs = [d for d in subsets if d.name in config.val_datasets]
        train_dirs = [d for d in subsets if d.name not in config.val_datasets]
        if not val_dirs:
            raise ValueError('split_mode: bag だが val_datasets が空')
        if not train_dirs:
            raise ValueError('val_datasets が全 bag を占めていて train が空')

        print('Train bags: ' + ', '.join(d.name for d in train_dirs))
        print('Val   bags: ' + ', '.join(d.name for d in val_dirs))

        stack = config.frame_stack_distances
        train_dataset = ConcatDataset(
            [E2EDataset(d, augment_config=aug_cfg, frame_stack_distances=stack,
                        use_pylon_mask=config.pylon_mask, draw_pylon_boxes=config.pylon_bbox_on_rgb,
                        past_mask_distances=config.past_mask_distances, past_mask_mode=config.past_mask_mode, use_pylon_scenes=config.pylon_scenes,
                        drop_backward_paths=config.drop_backward_paths,
                        holdout_ranges=config.holdout.get(d.name, []), holdout_gap=config.holdout_gap,
                        avoid_only=d.name in config.avoid_only_datasets,
                        blue_course_only=config.blue_course_only,
                        keep_avoid_off_course=config.keep_avoid_off_course,
                        extra_rgb_dirs=config.extra_rgb_dirs)
             for d in train_dirs])
        if config.extra_rgb_dirs:
            n_extra = sum(len(d.rgb_files) - sum(f.parent == d.rgb_images_dir for f in d.rgb_files)
                          for d in train_dataset.datasets)
            print(f'データ拡張の RGB: {n_extra} 件 (train {len(train_dataset)} 件のうち)')
        val_dataset = ConcatDataset(
            [E2EDataset(d, augment_config=None, frame_stack_distances=stack,
                        use_pylon_mask=config.pylon_mask, draw_pylon_boxes=config.pylon_bbox_on_rgb,
                        past_mask_distances=config.past_mask_distances, past_mask_mode=config.past_mask_mode, use_pylon_scenes=config.pylon_scenes,
                        drop_backward_paths=config.drop_backward_paths,
                        holdout_ranges=config.holdout.get(d.name, []), holdout_gap=config.holdout_gap,
                        avoid_only=d.name in config.avoid_only_datasets,
                        blue_course_only=config.blue_course_only,
                        keep_avoid_off_course=config.keep_avoid_off_course)
             for d in val_dirs])
        return train_dataset, val_dataset

    @staticmethod
    def _split_within_dataset(dataset_path, config, aug_cfg):
        stack = config.frame_stack_distances
        train_source = E2EDataset(dataset_path, augment_config=aug_cfg, frame_stack_distances=stack,
                                  use_pylon_mask=config.pylon_mask, draw_pylon_boxes=config.pylon_bbox_on_rgb,
                        past_mask_distances=config.past_mask_distances, past_mask_mode=config.past_mask_mode, use_pylon_scenes=config.pylon_scenes,
                        drop_backward_paths=config.drop_backward_paths,
                                  holdout_ranges=config.holdout.get(dataset_path.name, []),
                                  holdout_gap=config.holdout_gap,
                                  avoid_only=dataset_path.name in config.avoid_only_datasets,
                                  blue_course_only=config.blue_course_only,
                        keep_avoid_off_course=config.keep_avoid_off_course)
        val_source = E2EDataset(dataset_path, augment_config=None, frame_stack_distances=stack,
                                use_pylon_mask=config.pylon_mask, draw_pylon_boxes=config.pylon_bbox_on_rgb,
                        past_mask_distances=config.past_mask_distances, past_mask_mode=config.past_mask_mode, use_pylon_scenes=config.pylon_scenes,
                        drop_backward_paths=config.drop_backward_paths,
                                  holdout_ranges=config.holdout.get(dataset_path.name, []),
                                  holdout_gap=config.holdout_gap,
                                  avoid_only=dataset_path.name in config.avoid_only_datasets,
                                  blue_course_only=config.blue_course_only,
                        keep_avoid_off_course=config.keep_avoid_off_course)

        n = len(val_source)
        train_size = int(0.8 * n)

        if config.split_mode == 'temporal':
            # ファイル名は収集順の連番なので index 順 = 時系列順。
            # random_split だと 0.2s しか離れていないほぼ同一フレームが train と val に
            # 分かれて入り、val loss が汎化性能を過大評価する（リーク）。
            # 前半 80% を train、後半を val にして時間で切り離す。
            # さらに境界に gap を空ける: waypoint は 2.5s 先まで見るので、
            # 境界付近のフレームは train と未来の軌跡を共有してしまう
            gap = config.split_gap
            train_indices = list(range(train_size))
            val_indices = list(range(min(train_size + gap, n), n))
        else:
            split_generator = torch.Generator().manual_seed(SPLIT_SEED)
            tr_idx, va_idx = random_split(
                range(n), [train_size, n - train_size], generator=split_generator)
            train_indices, val_indices = list(tr_idx), list(va_idx)

        return Subset(train_source, train_indices), Subset(val_source, val_indices)

    def validate(self, loader: DataLoader = None) -> Tuple[float, dict]:
        """val loss と、正規化を戻した実距離[m]の誤差を返す。

        MSE は正規化後の値なので大きさの感覚が掴めない。実験前に「この重みで
        何 m ずれるのか」を見たいので、m に直した誤差も一緒に出して
        TensorBoard に載せる
        """
        self.model.eval()
        total_loss = 0.0
        x_error = 0.0
        y_error = 0.0
        endpoint_error = 0.0
        samples = 0
        # 急カーブだけの集計。曲がりきれずに縮むかどうかを見る
        tight_endpoint = 0.0
        tight_pred_lateral = 0.0
        tight_gt_lateral = 0.0
        tight_samples = 0

        with torch.no_grad():
            loader = loader or self.val_loader
            pbar = tqdm(loader, desc='Validation')
            for masks, rgbs, waypoints, _, scores in pbar:
                masks = masks.to(self.device)
                rgbs = rgbs.to(self.device)
                waypoints = waypoints.to(self.device)
                scores = scores.to(self.device)

                outputs = self.model(masks, rgbs)

                loss = self.mseloss(outputs, waypoints)
                total_loss += loss.item()

                # (B, NUM_WAYPOINTS*2) -> (B, NUM_WAYPOINTS, 2) にして m に戻す。
                # 正規化はスケール倍 + 平行移動なので、差分にはスケールだけ効く
                diff = (outputs - waypoints).view(-1, NUM_WAYPOINTS, 2)
                diff_m = diff * torch.tensor(
                    [WAYPOINT_X_SCALE, WAYPOINT_Y_SCALE], device=diff.device)

                batch = diff.shape[0]
                x_error += diff_m[:, :, 0].abs().mean(dim=1).sum().item()
                y_error += diff_m[:, :, 1].abs().mean(dim=1).sum().item()
                endpoint_error += diff_m[:, -1, :].norm(dim=1).sum().item()
                samples += batch

                tight = scores.abs() >= self.config.eval_tight_m
                if tight.any():
                    # 曲がる向きに射影した終端の横位置。予測/正解 が 1 未満なら縮んでいる
                    sign = torch.sign(scores[tight])
                    pred_y = (outputs.view(-1, NUM_WAYPOINTS, 2)[tight, -1, 1] + 1.0) * WAYPOINT_Y_SCALE - WAYPOINT_Y_OFFSET
                    gt_y = (waypoints.view(-1, NUM_WAYPOINTS, 2)[tight, -1, 1] + 1.0) * WAYPOINT_Y_SCALE - WAYPOINT_Y_OFFSET
                    tight_pred_lateral += (pred_y * sign).sum().item()
                    tight_gt_lateral += (gt_y * sign).sum().item()
                    tight_endpoint += diff_m[tight, -1, :].norm(dim=1).sum().item()
                    tight_samples += int(tight.sum().item())

                pbar.set_postfix({'loss': f'{loss.item():.6f}'})

        metrics = {
            'x_mae_m': x_error / samples,
            'y_mae_m': y_error / samples,
            'endpoint_m': endpoint_error / samples,
            'tight_endpoint_m': tight_endpoint / max(1, tight_samples),
            'tight_lateral_ratio': tight_pred_lateral / tight_gt_lateral if tight_gt_lateral else float('nan'),
            'tight_samples': tight_samples,
        }
        return total_loss / len(loader), metrics

    def save_checkpoint(self, val_loss: float) -> None:
        if val_loss < self.best_val_loss:
            self.best_val_loss = val_loss
            weight_path = self.config.weights_dir / self.config.weight_file
            was_training = self.model.training
            self.model.eval()
            scripted_model = torch.jit.script(self.model)
            scripted_model.save(str(weight_path))
            self.model.train(was_training)
            print(f'Best model saved: {weight_path} (val_loss: {val_loss:.6f})')

    def train(self, epochs: int) -> None:
        for epoch in range(1, epochs + 1):
            self.model.train()
            total_train_loss = 0.0
            total_grad_norm = 0.0
            # 重みなしの train loss。val loss / train.py の run と同じ尺度で比べる用
            total_train_plain = 0.0

            pbar = tqdm(self.train_loader, desc=f'Epoch {epoch} [Train]')
            for masks, rgbs, waypoints, weights, _ in pbar:
                masks = masks.to(self.device)
                rgbs = rgbs.to(self.device)
                waypoints = waypoints.to(self.device)
                weights = weights.to(self.device)

                self.optimizer.zero_grad()
                outputs = self.model(masks, rgbs)

                loss = self.weighted_mse(outputs, waypoints, weights)
                loss.backward()
                total_train_plain += self.mseloss(outputs.detach(), waypoints).item()

                # 勾配クリッピング。外れ値サンプルで学習が発散するのを防ぐ
                # （clip 無しでは epoch 60 付近で train 0.005 -> 0.050 に飛んだ）
                grad_norm = torch.nn.utils.clip_grad_norm_(
                    self.model.parameters(), self.config.grad_clip_norm
                )
                total_grad_norm += grad_norm.item()

                self.optimizer.step()

                total_train_loss += loss.item()
                pbar.set_postfix({'loss': f'{loss.item():.6f}'})

            train_loss = total_train_loss / len(self.train_loader)
            val_loss, metrics = self.validate()

            self.writer.add_scalar('Loss/train', train_loss, epoch)
            self.writer.add_scalar('Loss/train_unweighted', total_train_plain / len(self.train_loader), epoch)
            self.writer.add_scalar('Loss/val', val_loss, epoch)
            self.writer.add_scalar('GradNorm/train', total_grad_norm / len(self.train_loader), epoch)
            # 正規化を戻した実距離[m]。experiment 前に「何 m ずれるか」で判断できるようにする
            self.writer.add_scalar('ErrorMeters/val_x_mae', metrics['x_mae_m'], epoch)
            self.writer.add_scalar('ErrorMeters/val_y_mae', metrics['y_mae_m'], epoch)
            self.writer.add_scalar('ErrorMeters/val_endpoint', metrics['endpoint_m'], epoch)
            self.writer.add_scalar('CurveTight/val_endpoint', metrics['tight_endpoint_m'], epoch)
            self.writer.add_scalar('CurveTight/val_lateral_ratio', metrics['tight_lateral_ratio'], epoch)
            self.writer.add_scalar('LearningRate', self.optimizer.param_groups[0]['lr'], epoch)
            # 避ける区間だけの値 (ValAvoid は val の bag、Test/ は学習にも選択にも使わない評価用)
            for name, loader in self.avoid_loaders.items():
                loss, m = self.validate(loader)
                self.writer.add_scalar(f'{name}/loss', loss, epoch)
                self.writer.add_scalar(f'{name}/endpoint', m['endpoint_m'], epoch)
                self.writer.add_scalar(f'{name}/y_mae', m['y_mae_m'], epoch)
                self.writer.add_scalar(f'{name}/x_mae', m['x_mae_m'], epoch)
            self.writer.flush()

            if self.scheduler is not None:
                self.scheduler.step()

            print(f'Epoch [{epoch}/{epochs}], Train Loss: {train_loss:.6f}, Val Loss: {val_loss:.6f}, '
                  f'val 横ずれ {metrics["y_mae_m"]:.3f} m, 終端 {metrics["endpoint_m"]:.3f} m, '
                  f'急カーブ({metrics["tight_samples"]}件) 終端 {metrics["tight_endpoint_m"]:.3f} m '
                  f'曲がり {metrics["tight_lateral_ratio"]:.2f} 倍')

            self.save_checkpoint(val_loss)

        self.writer.close()

def main() -> None:
    if len(sys.argv) not in (2, 3, 4):
        print('Usage: python3 train_curve_weight.py <dataset_path> [run_name] [config_path]')
        sys.exit(1)

    dataset_path = Path(sys.argv[1])
    if not dataset_path.exists():
        print(f'Dataset path does not exist: {dataset_path}')
        sys.exit(1)

    run_name = sys.argv[2] if len(sys.argv) >= 3 else time.strftime('%Y%m%d_%H%M%S')

    script_dir = Path(__file__).parent
    package_root = script_dir.parent
    # 設定違いを並べて回せるよう、config を差し替えられるようにしておく
    config_path = (Path(sys.argv[3]) if len(sys.argv) == 4
                   else package_root / 'config' / 'train_curve_weight.yaml')

    config = Config(config_path, package_root, run_name)
    trainer = Trainer(dataset_path, config)

    print(f'Run name: {run_name}')
    print(f'TensorBoard logs: {config.logs_dir}')
    print(f'Weight file: {config.weights_dir / config.weight_file}')
    print(f'Starting training for {config.epochs} epochs')
    trainer.train(config.epochs)

    print('Training complete.')

if __name__ == '__main__':
    main()
