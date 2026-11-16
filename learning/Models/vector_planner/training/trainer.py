import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import ConcatDataset, DataLoader, WeightedRandomSampler

from Models.vector_planner.data.dataset import E2EDataset, split_datasets
from Models.vector_planner.data.frame_label import parse_lengths
from Models.vector_planner.data.online_aug import OnlinePaste, PoseShift
from Models.vector_planner.data.paste_aug import hue_swap
from Models.vector_planner.eval.metrics import path_error_sums
from Models.vector_planner.model.network import VectorPlannerNetwork
from Models.vector_planner.training.loss import DEFAULT_WEIGHTS, planner_loss

PATH_OFF = {'path': 0.0, 'path_valid': 0.0, 'clearance': 0.0, 'corridor': 0.0, 'branch_exist': 0.0,
            'agent_clearance': 0.0, 'boundary': 0.0, 'direction': 0.0}
AGENT_TERMS = ('agent_cls', 'agent_l1', 'agent_giou')
TREAD_M = 0.6
GRAD_CLIP = 10.0
STATE_CHECK_EVERY = 50


class NonFiniteLossError(RuntimeError):
    pass


def outputs_finite(outputs):
    flags = [torch.isfinite(v).all() for v in outputs.values() if torch.is_tensor(v) and v.is_floating_point()]
    return bool(torch.stack(flags).all()) if flags else True


def assert_finite_state(model):
    for kind, items in (('パラメータ', model.named_parameters()), ('バッファ', model.named_buffers())):
        for name, tensor in items:
            if tensor.is_floating_point() and not bool(torch.isfinite(tensor).all()):
                raise NonFiniteLossError(f'{kind}が非有限: {name}（復旧せず停止する）')


def parse_data_spec(spec):
    parts = spec.split(':')
    if len(parts) != 2 or not all(parts):
        raise ValueError(f'--data は IMAGE_ROOT:LABEL_ROOT 形式: {spec}')
    return parts[0], parts[1]


def parse_args(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--data', action='append', required=True, help='IMAGE_ROOT:LABEL_ROOT（複数指定可）')
    p.add_argument('--split-json', required=True)
    p.add_argument('--runs-dir', required=True)
    p.add_argument('--version', default='n')
    p.add_argument('--det-extra', action='append', default=[],
                   help='IMAGE_ROOT:LABEL_ROOT（複数指定可）。検出専用フレーム（labels/train、det_only）。地図・経路・分岐の損失は'
                        'マスクされ、online paste / 姿勢摂動 / ヨー回転は無効（左右反転は有効）')
    p.add_argument('--det-extra-ratio', type=float, default=0.1, help='--det-extra: 1 サンプルあたりの検出専用フレームの抽出確率')
    p.add_argument('--signal-head', action='store_true',
                   help='AgentHead に信号 query を 1 本足す（signal_logits 4 クラス + signal_xy）。既存 ckpt は --init-from で読める')
    p.add_argument('--signal-weight', type=float, default=1.0, help='--signal-head: 信号損失（CE + L1）全体の倍率')
    p.add_argument('--signal-only-root', action='append', default=[],
                   help='IMAGE_ROOT:LABEL_ROOT（複数指定可）。信号専用フレーム（labels/train、signal_only）。地図・経路・分岐の損失は'
                        'マスクされ、online paste / 姿勢摂動は無効（左右反転・ヨー回転は有効）')
    p.add_argument('--signal-only-ratio', type=float, default=0.1, help='--signal-only-root: 1 サンプルあたりの信号専用フレームの抽出確率')
    p.add_argument('--hue-swap-prob', type=float, default=0.0,
                   help='信号パネルの色相入れ替え（赤<->緑）の確率')
    p.add_argument('--geo-aug', action='store_true', help='左右反転・ヨー回転（train のみ）')
    p.add_argument('--avoid-ratio', type=float, default=None, help='straight=avoid サンプルの抽出比率（指定で WeightedRandomSampler）')
    p.add_argument('--avoid-weight', type=float, default=3.0, help='avoid スロットの path 損失の重み')
    p.add_argument('--clearance-weight', type=float, default=1.0, help='pylon クリアランス損失の重み')
    p.add_argument('--corridor-weight', type=float, default=1.0, help='走行帯 L1 損失の重み')
    p.add_argument('--branch-weight', type=float, default=1.0, help='branch_exist BCE の重み')
    p.add_argument('--agent-weight', type=float, default=1.0, help='agent（DETR: cls / L1 / GIoU）損失全体の倍率')
    p.add_argument('--agent-clearance-weight', type=float, default=0.2, help='推論 agent との衝突ヒンジの重み')
    p.add_argument('--boundary-weight', type=float, default=1.0, help='走行帯境界の越境ヒンジの重み')
    p.add_argument('--direction-weight', type=float, default=0.2, help='接線方向の cos ペナルティの重み')
    p.add_argument('--split-alpha', action='store_true', help='alpha を ego query 複製枝から計算する（center/half と分離）')
    p.add_argument('--init-from', default=None, help='初期化に使うチェックポイント（split_alpha の有無どちらでも可）')
    p.add_argument('--alpha-only', action='store_true',
                   help='alpha 枝と alpha 出力層以外を凍結（BN は eval 固定）。--split-alpha と --map-epochs 0 が必要')
    p.add_argument('--online-paste', action='store_true',
                   help='train でパイロンをオンライン貼り付け（事前生成 __p の aug フレームは train から除外。holdout は不変）')
    p.add_argument('--paste-osm', default=None, help='--online-paste: 地図 osm')
    p.add_argument('--paste-bank', default=None, help='--online-paste: スプライトバンク npz')
    p.add_argument('--paste-params', default=None, help='--online-paste: main_params.yaml（既定は repo のもの）')
    p.add_argument('--paste-prob', type=float, default=0.7, help='--online-paste: 1 サンプルあたりの貼り付け確率')
    p.add_argument('--paste-redraws', type=int, default=3, help='--online-paste: straight が stop になったときの引き直し回数')
    p.add_argument('--paste-group-prob', type=float, default=0.0, help='--online-paste: 横一列のコーン列（1〜4 個）を貼る確率')
    p.add_argument('--paste-group-sizes', default='1,2,3,4', help='--online-paste: コーン列の個数候補（一様）')
    p.add_argument('--paste-group-spacing', default='0.8,1.0', help='--online-paste: コーン列の隣接間隔 min,max [m]')
    p.add_argument('--paste-hue-prob', type=float, default=0.0, help='--online-paste: コーン列（グループ）ごとに色相をパレットから変える確率')
    p.add_argument('--paste-far-degrade-prob', type=float, default=0.0, help='--online-paste: 遠方（縮小後 24px 未満）コーンにぼかし＋ノイズをかける確率')
    p.add_argument('--paste-s-range', default=None, help='--online-paste: コーンを貼る基準線上の弧長 min,max [m]（例 1.5,18）。既定は paste_aug.S_RANGE')
    p.add_argument('--paste-person-prob', type=float, default=0.0, help='--online-paste: 人（cls1）を 1〜2 人加えて貼る確率（0 で無効）')
    p.add_argument('--paste-person-bank', default=None, help='--paste-person-prob>0: 人スプライトバンク npz（tools.vector_planner.sprites bank --cls 1）')
    p.add_argument('--paste-person-s-range', default=None, help='--online-paste: 人を貼る基準線上の弧長 min,max [m]。既定 1.5,15')
    p.add_argument('--paste-person-width', type=float, default=0.6, help='--online-paste: 人を教師の障害物とみなす幅 [m]')
    p.add_argument('--paste-pylon-height', type=float, default=0.70)
    p.add_argument('--paste-pylon-width', type=float, default=0.25)
    p.add_argument('--pose-shift-p', type=float, default=0.0,
                   help='姿勢摂動拡張を行う確率（0 で無効）。有効時はヨー回転拡張の代わりに使う（flip は継続）。--paste-osm が必要')
    p.add_argument('--pose-shift-dy', type=float, default=0.7, help='姿勢摂動の横ずれ範囲 U(-dy, dy) [m]')
    p.add_argument('--pose-shift-dyaw', type=float, default=6.0, help='姿勢摂動のヨー範囲 U(-d, d) [deg]')
    p.add_argument('--teacher-lengths', default='6',
                   help='online paste / 姿勢摂動で再生成する教師 Frenet の target_lengths（例 6 / 6,8）')
    p.add_argument('--max-nonfinite', type=int, default=20,
                   help='出力/損失が非有限のバッチをスキップし、連続でこの回数に達したら停止')
    p.add_argument('--map-epochs', type=int, default=10)
    p.add_argument('--freeze-epochs', type=int, default=10)
    p.add_argument('--joint-epochs', type=int, default=20)
    p.add_argument('--batch-size', type=int, default=16)
    p.add_argument('--lr', type=float, default=1e-3)
    p.add_argument('--weight-decay', type=float, default=1e-4)
    p.add_argument('--workers', type=int, default=4)
    p.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    p.add_argument('--input-height', type=int, default=384)
    p.add_argument('--input-width', type=int, default=640)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--max-batches', type=int, default=None,
                   help='1 epoch あたりの学習・検証の最大バッチ数（スモーク用）')
    return p.parse_args(argv)


def stage_of(epoch, args):
    if epoch < args.map_epochs:
        return 'map'
    return 'freeze' if epoch < args.map_epochs + args.freeze_epochs else 'joint'


def stage_weights(stage):
    return dict(PATH_OFF) if stage == 'map' else {}


def extra_weights(args):
    weights = {'avoid_weight': args.avoid_weight, 'clearance': args.clearance_weight, 'corridor': args.corridor_weight,
               'tread': TREAD_M, 'branch_exist': args.branch_weight, 'agent': args.agent_weight,
               'agent_clearance': args.agent_clearance_weight, 'boundary': args.boundary_weight,
               'direction': args.direction_weight}
    if args.signal_head:
        weights['signal'] = args.signal_weight
    weights.update({k: DEFAULT_WEIGHTS[k] * weights['agent'] for k in AGENT_TERMS})
    if args.alpha_only:
        weights.update({k: 0.0 for k in (*AGENT_TERMS, 'map_cls', 'map_points', 'corridor', 'branch_exist')})
    return weights


def parse_s_range(text):
    if text is None:
        return None
    try:
        values = tuple(float(v) for v in text.split(','))
    except ValueError:
        raise ValueError(f'--paste-s-range は数値 min,max: {text}')
    if len(values) != 2 or values[0] < 0 or values[0] >= values[1]:
        raise ValueError(f'--paste-s-range は 0 <= min < max の 2 要素: {text}')
    return values


def validate_args(args):
    parse_s_range(args.paste_s_range)
    if not 0.0 <= args.paste_person_prob <= 1.0:
        raise ValueError(f'--paste-person-prob は 0〜1: {args.paste_person_prob}')
    if args.paste_person_prob > 0.0 and not args.paste_person_bank:
        raise ValueError('--paste-person-prob > 0 には --paste-person-bank が必要')
    if args.paste_person_width <= 0.0:
        raise ValueError('--paste-person-width は正')
    parse_s_range(args.paste_person_s_range)
    if args.signal_only_root and not args.signal_head:
        raise ValueError('--signal-only-root には --signal-head が必要')
    if args.signal_head and args.signal_only_root and not 0.0 < args.signal_only_ratio < 1.0:
        raise ValueError('--signal-only-ratio は 0 より大きく 1 未満')
    if not 0.0 <= args.hue_swap_prob <= 1.0:
        raise ValueError('--hue-swap-prob は 0 以上 1 以下')
    if args.hue_swap_prob > 0 and not args.signal_head:
        raise ValueError('--hue-swap-prob には --signal-head が必要')
    if args.det_extra and not 0.0 < args.det_extra_ratio < 1.0:
        raise ValueError('--det-extra-ratio は 0 より大きく 1 未満')
    if args.alpha_only and not args.split_alpha:
        raise ValueError('--alpha-only には --split-alpha が必要')
    if args.alpha_only and args.map_epochs != 0:
        raise ValueError('--alpha-only では --map-epochs 0 が必要（map ステージは経路損失が 0 で学習対象がなくなる）')


def freeze_for_alpha_only(model):
    for name, param in model.named_parameters():
        param.requires_grad = model.is_alpha_param(name)


def trainable_parameters(model):
    return [p for p in model.parameters() if p.requires_grad]


def set_train_mode(model, alpha_only):
    # alpha_only では凍結部分（BatchNorm 等）を eval に保つ
    model.train()
    if alpha_only:
        for name, module in model.named_children():
            if name != 'ego_head':
                module.eval()
        model.ego_head.eval()
        for name, module in model.ego_head.named_children():
            if name.startswith('alpha'):
                module.train()


SIGNAL_PREFIX = 'agent_head.signal_'


def load_init_checkpoint(model, path):
    state = torch.load(path, map_location='cpu')['model']
    has_alpha_branch = any(k.startswith('ego_head.alpha_') for k in state)
    if has_alpha_branch and not model.split_alpha:
        raise ValueError('split_alpha のチェックポイントは split_alpha なしのモデルへ読めない')
    saved_signal = any(k.startswith(SIGNAL_PREFIX) for k in state)
    if saved_signal and not model.signal_head:
        raise ValueError('signal_head のチェックポイントは signal_head なしのモデルへ読めない')
    new_signal = model.signal_head and not saved_signal
    new_split = model.split_alpha and not has_alpha_branch
    if new_split or new_signal:
        result = model.load_state_dict(state, strict=False)
        if result.unexpected_keys or not all(model.is_alpha_param(k) or k.startswith(SIGNAL_PREFIX)
                                             for k in result.missing_keys):
            raise ValueError(f'init-from の不整合: {result}')
        if new_split:
            model.init_alpha_branch_from_shared()
    else:
        model.load_state_dict(state)


def build_model(args):
    return VectorPlannerNetwork(args.version, input_size=(args.input_height, args.input_width),
                                split_alpha=args.split_alpha, signal_head=args.signal_head)


AVOID, STOP, OTHER = 'avoid', 'stop', 'other'


def _category(label):
    entry = label.get('paths', {}).get('straight')
    status = None if entry is None else entry.get('status')
    return status if status in (AVOID, STOP) else OTHER


def sample_categories(datasets):
    categories = []
    for ds in datasets:
        for i in range(len(ds)):
            categories.append(_category(ds.read_label(i)))
    return categories


def category_weights(categories, avoid_ratio):
    # 各カテゴリの合計重みが目標比率になる per-sample 重み
    categories = np.asarray(categories)
    counts = {c: int((categories == c).sum()) for c in (AVOID, STOP, OTHER)}
    natural = {c: counts[c] / max(len(categories), 1) for c in counts}
    avoid_ratio = natural[AVOID] if avoid_ratio is None else avoid_ratio
    targets = {AVOID: avoid_ratio, STOP: natural[STOP], OTHER: max(0.0, 1.0 - avoid_ratio - natural[STOP])}
    present = {c: t for c, t in targets.items() if counts[c] > 0}
    total = sum(present.values())
    weights = np.zeros(len(categories))
    for c, t in present.items():
        weights[categories == c] = (t / total) / counts[c]
    return weights.tolist()


def make_sampler(datasets, avoid_ratio, num_samples=None, seed=0, extra=None, extra_ratio=0.0, extra_groups=None):
    # extra: 検出専用データセット群（ConcatDataset 上で datasets の後ろに並べる）
    categories = sample_categories(datasets)
    main = np.asarray(category_weights(categories, avoid_ratio))
    main = main / max(main.sum(), 1e-12)
    n_main = len(categories)
    groups = ([(extra, extra_ratio)] if extra else []) + [(list(g), r) for g, r in (extra_groups or []) if g]
    if groups:
        total = sum(r for _, r in groups)
        parts = [main * (1.0 - total)]
        for sets, ratio in groups:
            n = sum(len(d) for d in sets)
            parts.append(np.full(n, ratio / max(n, 1)))
        main = np.concatenate(parts)
    weights = torch.tensor(main, dtype=torch.double)
    generator = torch.Generator().manual_seed(seed)
    return WeightedRandomSampler(weights, num_samples or n_main, replacement=True, generator=generator)


def build_datasets(args, specs, split_map):
    size = (args.input_height, args.input_width)
    kwargs = {}
    if args.online_paste:
        if not (args.paste_osm and args.paste_bank):
            raise ValueError('--online-paste には --paste-osm と --paste-bank が必要')
        paste_kwargs = {} if args.paste_params is None else {'params_path': args.paste_params}
        kwargs = {'online_paste': OnlinePaste(args.paste_osm, args.paste_bank, prob=args.paste_prob,
                                              pylon_height=args.paste_pylon_height, pylon_width=args.paste_pylon_width,
                                              redraws=args.paste_redraws, seed=None,
                                              group_prob=args.paste_group_prob, hue_prob=args.paste_hue_prob,
                                              far_degrade_prob=args.paste_far_degrade_prob,
                                              group_sizes=tuple(int(v) for v in args.paste_group_sizes.split(',')),
                                              group_spacing=tuple(float(v) for v in args.paste_group_spacing.split(',')),
                                              s_range=parse_s_range(args.paste_s_range),
                                              person_prob=args.paste_person_prob, person_bank_path=args.paste_person_bank,
                                              person_s_range=parse_s_range(args.paste_person_s_range),
                                              person_width=args.paste_person_width,
                                              teacher_lengths=parse_lengths(args.teacher_lengths), **paste_kwargs),
                  'stem_filter': lambda stem: '__p' not in stem}
    if args.pose_shift_p > 0:
        if not args.paste_osm:
            raise ValueError('--pose-shift-p には --paste-osm が必要')
        shift_kwargs = {} if args.paste_params is None else {'params_path': args.paste_params}
        kwargs['pose_shift'] = PoseShift(args.paste_osm, p=args.pose_shift_p, dy_max=args.pose_shift_dy,
                                         dyaw_max_deg=args.pose_shift_dyaw,
                                         teacher_lengths=parse_lengths(args.teacher_lengths), **shift_kwargs)
    train = split_datasets(specs, split_map, 'train', size, geo_aug=args.geo_aug, **kwargs)
    hold = split_datasets(specs, split_map, 'holdout', size)
    return train, hold


def build_det_extra(args):
    size = (args.input_height, args.input_width)
    sets = []
    for spec in args.det_extra:
        image_root, label_root = parse_data_spec(spec)
        dataset = E2EDataset(image_root, label_root, 'train', size, geo_aug=args.geo_aug)
        if len(dataset):
            sets.append(dataset)
    return sets


def build_signal_only(args):
    size = (args.input_height, args.input_width)
    hue = hue_swap if args.hue_swap_prob > 0 else None
    sets = []
    for spec in args.signal_only_root:
        image_root, label_root = parse_data_spec(spec)
        dataset = E2EDataset(image_root, label_root, 'train', size, geo_aug=args.geo_aug, hue_swap=hue,
                             hue_swap_prob=args.hue_swap_prob)
        if len(dataset):
            sets.append(dataset)
    return sets


def photometric(image, generator):
    b = image.shape[0]
    brightness = (0.75 + 0.5 * torch.rand(b, generator=generator)).to(image.device).view(b, 1, 1, 1)
    contrast = (0.75 + 0.5 * torch.rand(b, generator=generator)).to(image.device).view(b, 1, 1, 1)
    mean = image.mean(dim=(1, 2, 3), keepdim=True)
    return (((image - mean) * contrast + mean) * brightness).clamp(0.0, 1.0)


def to_device(batch, device):
    return {k: v.to(device, non_blocking=True) for k, v in batch.items()}


def make_loader(datasets, args, train, extra=(), signal_only=()):
    dataset = ConcatDataset([*datasets, *extra, *signal_only])
    sampler = None
    if train and (extra or signal_only or args.avoid_ratio is not None):
        groups = [(list(signal_only), args.signal_only_ratio)] if signal_only else None
        sampler = make_sampler(datasets, args.avoid_ratio, seed=args.seed, extra=list(extra),
                               extra_ratio=args.det_extra_ratio if extra else 0.0, extra_groups=groups)
    return DataLoader(dataset, batch_size=args.batch_size, shuffle=train and sampler is None, sampler=sampler,
                      drop_last=train and len(dataset) > args.batch_size, num_workers=args.workers,
                      pin_memory=torch.device(args.device).type == 'cuda')


def limited(loader, max_batches):
    for i, batch in enumerate(loader):
        if max_batches is not None and i >= max_batches:
            return
        yield batch


def run_train_epoch(model, loader, optimizer, scheduler, scaler, generator, weights, args, device, state=None):
    # state: エポックをまたぐ連続非有限カウンタ。省略時は epoch 内のみ
    set_train_mode(model, args.alpha_only)
    sums, n, steps = {}, 0, 0
    skipped = 0
    state = {'consecutive': 0} if state is None else state
    max_nonfinite = args.max_nonfinite
    amp = device.type == 'cuda'

    def skip(reason):
        nonlocal skipped
        skipped += 1
        state['consecutive'] += 1
        print(f'WARNING: 非有限のためバッチをスキップ ({reason}) epoch 内累計 {skipped} / 連続 {state["consecutive"]}',
              file=sys.stderr, flush=True)
        assert_finite_state(model)
        if state['consecutive'] >= max_nonfinite:
            raise NonFiniteLossError(f'非有限が連続 {state["consecutive"]} 回（上限 {max_nonfinite}）')

    for batch in limited(loader, args.max_batches):
        batch = to_device(batch, device)
        image = photometric(batch['image'].float() / 255.0, generator)
        optimizer.zero_grad(set_to_none=True)
        try:
            with torch.autocast(device.type, dtype=torch.float16, enabled=amp):
                outputs = model(image, gt=batch)
            outputs = {k: v.float() for k, v in outputs.items()}
            if not outputs_finite(outputs):
                skip('出力')
                continue
            total, parts = planner_loss(outputs, batch, weights)
        except FloatingPointError as error:
            skip(str(error))
            continue
        if not torch.isfinite(total):
            skip(f'損失 {float(total)}')
            continue
        state['consecutive'] = 0
        scaler.scale(total).backward()
        scaler.unscale_(optimizer)
        torch.nn.utils.clip_grad_norm_(trainable_parameters(model), GRAD_CLIP)
        scaler.step(optimizer)
        scaler.update()
        scheduler.step()
        steps += 1
        if steps % STATE_CHECK_EVERY == 0:
            assert_finite_state(model)
        size = image.shape[0]
        for k, v in {'total': total.detach(), **parts}.items():
            sums[k] = sums.get(k, 0.0) + float(v) * size
        n += size
    assert_finite_state(model)
    if steps == 0 and skipped > 0:
        raise NonFiniteLossError(f'epoch の全 {skipped} バッチが非有限')
    return {**{k: v / max(n, 1) for k, v in sums.items()}, 'skipped_nonfinite': skipped}


@torch.no_grad()
def run_holdout(model, loader, args, device):
    model.eval()
    amp = device.type == 'cuda'
    totals = {'l1': 0.0, 'lateral': 0.0, 'count': 0}
    for batch in limited(loader, args.max_batches):
        batch = to_device(batch, device)
        with torch.autocast(device.type, dtype=torch.float16, enabled=amp):
            outputs = model(batch['image'].float() / 255.0)
        sums = path_error_sums(outputs['path_xy'].float(), batch['path_xy'], batch['path_mask'])
        for k in totals:
            totals[k] += sums[k]
    count = totals['count']
    if count == 0:
        return {'path_l1': None, 'path_lateral': None, 'points': 0}
    return {'path_l1': totals['l1'] / (2 * count), 'path_lateral': totals['lateral'] / count, 'points': count}


def save_checkpoint(path, model, epoch, args, holdout):
    torch.save({'model': model.state_dict(), 'epoch': epoch, 'version': args.version, 'holdout': holdout,
                'args': vars(args)}, path)


def train(args):
    validate_args(args)
    torch.manual_seed(args.seed)
    device = torch.device(args.device)
    split_map = json.loads(Path(args.split_json).read_text())
    specs = [parse_data_spec(s) for s in args.data]
    train_sets, hold_sets = build_datasets(args, specs, split_map)
    if not train_sets:
        raise ValueError('train フレームがありません')
    extra_sets = build_det_extra(args)
    signal_sets = build_signal_only(args)
    train_loader = make_loader(train_sets, args, True, extra_sets, signal_sets)
    hold_loader = make_loader(hold_sets, args, False) if hold_sets else None

    epochs = args.map_epochs + args.freeze_epochs + args.joint_epochs
    steps = len(train_loader) if args.max_batches is None else min(len(train_loader), args.max_batches)
    model = build_model(args)
    if args.init_from:
        load_init_checkpoint(model, args.init_from)
    if args.alpha_only:
        freeze_for_alpha_only(model)
    model = model.to(device)
    optimizer = torch.optim.AdamW(trainable_parameters(model), lr=args.lr, weight_decay=args.weight_decay)
    scheduler = torch.optim.lr_scheduler.OneCycleLR(optimizer, max_lr=args.lr, total_steps=max(epochs * steps, 1))
    scaler = torch.amp.GradScaler(device.type, enabled=device.type == 'cuda')
    generator = torch.Generator().manual_seed(args.seed)

    runs = Path(args.runs_dir)
    runs.mkdir(parents=True, exist_ok=True)
    (runs / 'args.json').write_text(json.dumps(vars(args), indent=1))
    best, best_epoch = float('inf'), None
    guard = {'consecutive': 0}
    for epoch in range(epochs):
        started = time.time()
        stage = stage_of(epoch, args)
        model.freeze_map = stage == 'freeze'
        train_log = run_train_epoch(model, train_loader, optimizer, scheduler, scaler, generator,
                                    {**extra_weights(args), **stage_weights(stage)}, args, device, guard)
        holdout = run_holdout(model, hold_loader, args, device) if hold_loader else {'path_l1': None}
        record = {'epoch': epoch + 1, 'stage': stage, 'freeze_map': model.freeze_map,
                  'lr': scheduler.get_last_lr()[0], 'train': train_log, 'holdout': holdout,
                  'time_s': time.time() - started}
        (runs / f'epoch_{epoch + 1:03d}.json').write_text(json.dumps(record, indent=1))
        save_checkpoint(runs / 'last.pt', model, epoch + 1, args, holdout)
        if stage != 'map':
            score = holdout['path_l1'] if holdout['path_l1'] is not None else -float(epoch)
            if score < best:
                best, best_epoch = score, epoch + 1
                save_checkpoint(runs / 'best.pt', model, epoch + 1, args, holdout)
        print(json.dumps({k: record[k] for k in ('epoch', 'stage', 'train', 'holdout')}), flush=True)
    return {'best_epoch': best_epoch, 'best_holdout_path_l1': None if best_epoch is None else best,
            'train_frames': sum(len(d) for d in train_sets), 'det_extra_frames': sum(len(d) for d in extra_sets),
            'signal_only_frames': sum(len(d) for d in signal_sets), 'holdout_frames': sum(len(d) for d in hold_sets)}


def main(argv=None):
    args = parse_args(argv)
    try:
        summary = train(args)
    except NonFiniteLossError as error:
        print(f'ERROR: {error}', file=sys.stderr)
        return 1
    print(json.dumps(summary))
    return 0


if __name__ == '__main__':
    sys.exit(main())
