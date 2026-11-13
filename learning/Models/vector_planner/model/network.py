import numpy as np
import torch
from torch import nn

from Models.common.backbone import Backbone
from Models.common.neck import Neck
from Models.vector_planner.data import geo_aug
from Models.vector_planner.model.heads import (DIM, NUM_DET_CLASSES, REF_HEIGHT, REF_WIDTH, AgentHead, EgoHead, MapHead,
                                               sine_position_encoding)
from Models.vision_planner.model.vision_planner_network import VisionPlannerNetwork

BEV_STEP = 0.5
BEV_X_MIN, BEV_Y_MIN = 0.0, -8.0
BEV_ROWS, BEV_COLS = 50, 32
BEV_TOKENS = BEV_ROWS * BEV_COLS
NUM_MAP_FEATURES = 20 * 2 + 4


class IpmSampler(nn.Module):
    # 固定 IPM。BEV 格子の地面点をカメラへ投影し、4 近傍の整数 index と固定重みで gather する
    def __init__(self, camera, feat_hw):
        super().__init__()
        height, width = feat_hw
        xs = BEV_X_MIN + BEV_STEP * (np.arange(BEV_ROWS) + 0.5)
        ys = BEV_Y_MIN + BEV_STEP * (np.arange(BEV_COLS) + 0.5)
        ground = np.stack([np.repeat(xs, BEV_COLS), np.tile(ys, BEV_ROWS), np.zeros(BEV_TOKENS)], axis=1)
        pixels, visible = camera.project(ground)
        fu = pixels[:, 0] / REF_WIDTH * width - 0.5
        fv = pixels[:, 1] / REF_HEIGHT * height - 0.5
        valid = visible & (fu >= 0) & (fu <= width - 1) & (fv >= 0) & (fv <= height - 1)
        fu, fv = np.where(valid, fu, 0.0), np.where(valid, fv, 0.0)
        u0 = np.clip(np.floor(fu), 0, width - 2).astype(np.int64)
        v0 = np.clip(np.floor(fv), 0, height - 2).astype(np.int64)
        du, dv = fu - u0, fv - v0
        idx = np.stack([v0 * width + u0, v0 * width + u0 + 1, (v0 + 1) * width + u0, (v0 + 1) * width + u0 + 1])
        weight = np.stack([(1 - du) * (1 - dv), du * (1 - dv), (1 - du) * dv, du * dv]) * valid
        self.register_buffer('idx', torch.from_numpy(idx))
        self.register_buffer('weight', torch.from_numpy(weight).float())
        self.register_buffer('valid', torch.from_numpy(valid.astype(np.float32)))

    def forward(self, feat):
        b, c = feat.shape[:2]
        flat = feat.flatten(2)
        picked = flat[:, :, self.idx.reshape(-1)].view(b, c, 4, BEV_TOKENS)
        return (picked * self.weight.to(feat.dtype)).sum(dim=2)


class BevEncoder(nn.Module):
    def __init__(self, camera, p2_hw, p3_hw, p2_channels, p3_channels):
        super().__init__()
        self.project2 = nn.Conv2d(p2_channels, DIM, 1)
        self.project3 = nn.Conv2d(p3_channels, DIM, 1)
        self.ipm2 = IpmSampler(camera, p2_hw)
        self.ipm3 = IpmSampler(camera, p3_hw)
        self.outside = nn.Parameter(torch.zeros(DIM))

        def block():
            return nn.Sequential(nn.Conv2d(DIM, DIM, 3, padding=1, bias=False), nn.BatchNorm2d(DIM), nn.SiLU())
        self.convs = nn.Sequential(block(), block())
        self.position = nn.Parameter(torch.randn(1, BEV_TOKENS, DIM) * 0.02)

    def forward(self, p2, p3):
        bev = self.ipm2(self.project2(p2)) + self.ipm3(self.project3(p3))
        bev = bev + (1.0 - self.ipm3.valid)[None, None, :] * self.outside[None, :, None]
        bev = self.convs(bev.view(bev.shape[0], DIM, BEV_ROWS, BEV_COLS))
        return bev.flatten(2).transpose(1, 2) + self.position


class VectorPlannerNetwork(nn.Module):
    def __init__(self, version='n', freeze_map=False, camera=None, input_size=(384, 640), use_bev_in_ego=True,
                 split_alpha=False, signal_head=False):
        super().__init__()
        self.signal_head = bool(signal_head)
        camera = camera or geo_aug.load_aug_camera()
        weighting = VisionPlannerNetwork.DYNAMIC_WEIGHTING[version]
        width, depth, csp = weighting['width'], weighting['depth'], weighting['csp']
        self.input_size = tuple(input_size)
        height, wide = self.input_size
        self.backbone = Backbone(width, depth, csp)
        self.neck = Neck(width, depth, csp)
        self.project = nn.Conv2d(width[3], DIM, 1)
        self.bev = BevEncoder(camera, (height // 4, wide // 4), (height // 8, wide // 8), width[3], width[3])
        self.map_head = MapHead()
        self.agent_head = AgentHead(camera, self.signal_head)
        self.map_embed = nn.Linear(NUM_MAP_FEATURES, DIM)
        self.split_alpha = split_alpha
        self.ego_head = EgoHead(use_bev_in_ego, split_alpha)
        self.freeze_map = freeze_map

    def init_alpha_branch_from_shared(self):
        self.ego_head.init_alpha_branch_from_shared()

    @staticmethod
    def is_alpha_param(name):
        return name.startswith('ego_head.alpha')

    def forward(self, image, gt=None):
        if tuple(image.shape[-2:]) != self.input_size:
            raise ValueError(f'入力サイズ {tuple(image.shape[-2:])} はモデルの固定サイズ {self.input_size} と異なる')
        p2, p3 = self.neck(self.backbone(image))
        bev_tokens = self.bev(p2, p3)
        feat = self.project(p3)
        _, _, h, w = feat.shape
        tokens = feat.flatten(2).transpose(1, 2) + sine_position_encoding(h, w, DIM, feat.device, feat.dtype)

        map_emb, map_logits, map_points = self.map_head(bev_tokens)
        agent_out, agent_tokens = self.agent_head(tokens, gt)

        parts = (map_emb, map_points.flatten(2), map_logits.softmax(-1))
        parts = tuple(p.detach() for p in parts) if self.freeze_map else parts
        map_tokens = parts[0] + self.map_embed(torch.cat(parts[1:], dim=-1))
        ego = self.ego_head(agent_tokens, map_tokens, bev_tokens)

        prob = agent_out['agent_logits'].softmax(-1)[..., :NUM_DET_CLASSES]
        return {'map_logits': map_logits, 'map_points': map_points, **agent_out, **ego,
                'det_boxes': agent_out['agent_boxes'], 'det_scores': prob.max(-1).values, 'det_cls': prob.argmax(-1)}


class ExportWrapper(nn.Module):
    OUTPUT_NAMES = ('map_prob', 'map_points', 'agent_boxes', 'agent_scores', 'agent_cls', 'agent_ground_xy',
                    'branch_exist', 'path_xy', 'path_valid', 'corridor_left', 'corridor_right')
    SIGNAL_OUTPUT_NAMES = ('signal_logits', 'signal_prob', 'signal_xy')

    def __init__(self, model):
        super().__init__()
        self.model = model
        self.with_signal = model.signal_head
        self.output_names = list(self.OUTPUT_NAMES) + (list(self.SIGNAL_OUTPUT_NAMES) if self.with_signal else [])

    def forward(self, image):
        o = self.model(image)
        outs = (o['map_logits'].softmax(-1), o['map_points'], o['det_boxes'], o['det_scores'], o['det_cls'],
                o['agent_ground_xy'], o['branch_exist'].sigmoid(), o['path_xy'], o['path_valid'].sigmoid(),
                o['corridor_left'], o['corridor_right'])
        if self.with_signal:
            outs = outs + (o['signal_logits'], o['signal_logits'].softmax(-1), o['signal_xy'])
        return outs


def load_network(checkpoint, device='cpu'):
    ckpt = torch.load(checkpoint, map_location='cpu')
    saved = ckpt.get('args') or {}
    size = (saved.get('input_height', 384), saved.get('input_width', 640))
    model = VectorPlannerNetwork(ckpt.get('version', 'n'), input_size=size, split_alpha=bool(saved.get('split_alpha', False)),
                                 signal_head=any(k.startswith('agent_head.signal_') for k in ckpt['model']))
    model.load_state_dict(ckpt['model'])
    return model.to(device).eval(), ckpt
