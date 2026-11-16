import math

import torch
from scipy.optimize import linear_sum_assignment
from torch import nn

from Models.vector_planner.data import geo_aug

NUM_MAP_QUERIES = 30
NUM_MAP_POINTS = 20
NUM_MAP_CLASSES = 3
NUM_DET_CLASSES = 4
NUM_SLOTS = 3
NUM_PATH_POINTS = 40
NUM_AGENT_QUERIES = 15
AGENT_NO_OBJECT = NUM_DET_CLASSES
NUM_SIGNAL_CLASSES = 4
DIM = 128

REF_WIDTH, REF_HEIGHT = geo_aug.REF_WIDTH, geo_aug.REF_HEIGHT
GROUND_X_SCALE, GROUND_Y_SCALE = 32.0, 16.0
GROUND_FREQS = 6
GROUND_LIMITS = ((-5.0, 60.0), (-20.0, 20.0))
MAX_RAY_SCALE = 80.0
SIGNAL_X_BIAS, SIGNAL_X_SCALE, SIGNAL_Y_SCALE = 12.0, 10.0, 3.0
PATH_STEP = 0.5
STEP_BIAS = 4.0
HALF_WIDTH_MIN = 0.2
HALF_WIDTH_BIAS = 1.3
TANGENT_EPS = 1e-12
MATCH_CLS_WEIGHT, MATCH_L1_WEIGHT, MATCH_GIOU_WEIGHT = 2.0, 5.0, 2.0


def sine_position_encoding(height, width, dim, device, dtype):
    quarter = dim // 4
    freq = 1.0 / (10000.0 ** (torch.arange(quarter, device=device, dtype=dtype) / quarter))
    ys = torch.arange(height, device=device, dtype=dtype)[:, None] * freq[None] * (2 * math.pi / height)
    xs = torch.arange(width, device=device, dtype=dtype)[:, None] * freq[None] * (2 * math.pi / width)
    pe_y = torch.cat([ys.sin(), ys.cos()], dim=1)[:, None, :].expand(height, width, dim // 2)
    pe_x = torch.cat([xs.sin(), xs.cos()], dim=1)[None, :, :].expand(height, width, dim // 2)
    return torch.cat([pe_y, pe_x], dim=2).reshape(height * width, dim)


def decoder_layer():
    return nn.TransformerDecoderLayer(DIM, 8, 256, dropout=0.0, batch_first=True, norm_first=True)


def make_decoder(layers=3):
    return nn.TransformerDecoder(decoder_layer(), layers)


def mlp(inp, hidden, out):
    return nn.Sequential(nn.Linear(inp, hidden), nn.ReLU(), nn.Linear(hidden, out))


def fourier(xy):
    scaled = torch.stack([xy[..., 0] / GROUND_X_SCALE, xy[..., 1] / GROUND_Y_SCALE], dim=-1)
    freq = (2.0 ** torch.arange(GROUND_FREQS, device=xy.device, dtype=xy.dtype)) * math.pi
    angle = scaled[..., None] * freq
    return torch.cat([angle.sin().flatten(-2), angle.cos().flatten(-2)], dim=-1)


def box_giou(a, b):
    # cx,cy,w,h 形式。先頭次元はブロードキャスト
    a_min, a_max = a[..., :2] - a[..., 2:] / 2, a[..., :2] + a[..., 2:] / 2
    b_min, b_max = b[..., :2] - b[..., 2:] / 2, b[..., :2] + b[..., 2:] / 2
    inter = (torch.minimum(a_max, b_max) - torch.maximum(a_min, b_min)).clamp(min=0).prod(-1)
    union = a[..., 2:].prod(-1) + b[..., 2:].prod(-1) - inter
    hull = (torch.maximum(a_max, b_max) - torch.minimum(a_min, b_min)).prod(-1)
    return inter / union.clamp(min=1e-9) - (hull - union) / hull.clamp(min=1e-9)


def match_agents(logits, boxes, batch):
    # サンプルごとのハンガリアン割当。(行=クエリ, 列=GT スロット番号) の組のリスト
    prob = logits.detach().softmax(-1)
    boxes = boxes.detach()
    pairs = []
    empty = torch.zeros(0, dtype=torch.long, device=logits.device)
    for i in range(logits.shape[0]):
        gt = batch['box_mask'][i].nonzero().squeeze(1) if bool(batch['has_boxes'][i]) else empty
        if len(gt) == 0:
            pairs.append((empty, empty))
            continue
        gt_boxes = batch['boxes'][i, gt]
        cost = (MATCH_L1_WEIGHT * torch.cdist(boxes[i], gt_boxes, p=1)
                - MATCH_CLS_WEIGHT * prob[i][:, batch['box_cls'][i, gt]]
                - MATCH_GIOU_WEIGHT * box_giou(boxes[i][:, None], gt_boxes[None]))
        if not bool(torch.isfinite(cost).all()):
            raise FloatingPointError('agent マッチングのコストが非有限（モデル出力に NaN/inf）')
        rows, cols = linear_sum_assignment(cost.cpu().numpy())
        pairs.append((torch.as_tensor(rows, device=logits.device), gt[torch.as_tensor(cols, device=logits.device)]))
    return pairs


class MapHead(nn.Module):
    def __init__(self):
        super().__init__()
        self.query = nn.Embedding(NUM_MAP_QUERIES, DIM)
        self.decoder = make_decoder()
        self.cls = nn.Linear(DIM, NUM_MAP_CLASSES + 1)
        self.points = mlp(DIM, DIM, NUM_MAP_POINTS * 2)

    def forward(self, memory):
        b = memory.shape[0]
        emb = self.decoder(self.query.weight[None].expand(b, -1, -1), memory)
        points = self.points(emb).sigmoid().view(b, NUM_MAP_QUERIES, NUM_MAP_POINTS, 2)
        return emb, self.cls(emb), points


class AgentHead(nn.Module):
    def __init__(self, camera, signal_head=False):
        super().__init__()
        self.signal_head = bool(signal_head)
        self.query = nn.Embedding(NUM_AGENT_QUERIES, DIM)
        self.decoder = make_decoder(2)
        if self.signal_head:
            self.signal_query = nn.Embedding(1, DIM)
            self.signal_cls = mlp(DIM, DIM, NUM_SIGNAL_CLASSES)
            self.signal_xy = mlp(DIM, DIM, 2)
        self.cls = nn.Linear(DIM, NUM_DET_CLASSES + 1)
        self.box = mlp(DIM, DIM, 4)
        self.ground_embed = nn.Linear(4 * GROUND_FREQS, DIM)
        self.prob_embed = nn.Linear(NUM_DET_CLASSES + 1, DIM)
        self.register_buffer('cam_k', torch.tensor([camera.fx, camera.fy, camera.cx, camera.cy], dtype=torch.float32))
        self.register_buffer('cam_r', torch.tensor(camera.rotation, dtype=torch.float32))
        self.register_buffer('cam_t', torch.tensor(camera.translation, dtype=torch.float32))
        self.tf_prob = 0.5

    def ground_xy(self, boxes):
        # 箱の下端中心を地面（z=0）へ逆投影。1/down を含むので AMP 下でも fp32
        with torch.autocast(device_type=boxes.device.type, enabled=False):
            boxes = boxes.float()
            u = boxes[..., 0] * REF_WIDTH
            v = (boxes[..., 1] + boxes[..., 3] / 2) * REF_HEIGHT
            ray = torch.stack([(u - self.cam_k[2]) / self.cam_k[0], (v - self.cam_k[3]) / self.cam_k[1],
                               torch.ones_like(u)], dim=-1)
            direction = ray @ self.cam_r.T
            down = direction[..., 2].clamp(max=-1e-3)
            scale = (-self.cam_t[2] / down).clamp(max=MAX_RAY_SCALE)
            x = self.cam_t[0] + scale * direction[..., 0]
            y = self.cam_t[1] + scale * direction[..., 1]
            return torch.stack([x.clamp(*GROUND_LIMITS[0]), y.clamp(*GROUND_LIMITS[1])], dim=-1)

    def teacher_force(self, ground, logits, boxes, gt):
        # マッチしたクエリの地面座標を、確率 tf_prob で GT 箱の座標へ差し替える
        forced = ground.clone()
        use = torch.rand(ground.shape[0], device=ground.device) < self.tf_prob
        for i, (rows, cols) in enumerate(match_agents(logits, boxes, gt)):
            if bool(use[i]) and len(rows):
                forced[i, rows] = self.ground_xy(gt['boxes'][i, cols])
        return forced

    def forward(self, tokens, gt=None):
        b = tokens.shape[0]
        queries = self.query.weight
        if self.signal_head:
            queries = torch.cat([queries, self.signal_query.weight], dim=0)
        emb = self.decoder(queries[None].expand(b, -1, -1), tokens)
        signal_emb = None
        if self.signal_head:
            emb, signal_emb = emb[:, :NUM_AGENT_QUERIES], emb[:, NUM_AGENT_QUERIES]
        logits = self.cls(emb)
        boxes = self.box(emb).sigmoid()
        ground = self.ground_xy(boxes.detach())
        if self.training and gt is not None and self.tf_prob > 0:
            ground = self.teacher_force(ground, logits.detach(), boxes.detach(), gt)
        agent_tokens = emb + self.ground_embed(fourier(ground.detach())) + self.prob_embed(logits.detach().softmax(-1))
        out = {'agent_logits': logits, 'agent_boxes': boxes, 'agent_ground_xy': ground}
        if signal_emb is not None:
            out.update(self.signal_outputs(signal_emb))
        return out, agent_tokens

    def signal_outputs(self, emb):
        # signal_xy はパネル直下の地面点（base_link の x 前方・y 左）。座標は fp32 で組む
        raw = self.signal_xy(emb)
        with torch.autocast(device_type=raw.device.type, enabled=False):
            raw = raw.float()
            xy = torch.stack([SIGNAL_X_BIAS + SIGNAL_X_SCALE * raw[..., 0], SIGNAL_Y_SCALE * raw[..., 1]], dim=-1)
        return {'signal_logits': self.signal_cls(emb), 'signal_xy': xy}


class EgoHead(nn.Module):
    def __init__(self, use_bev, split_alpha=False):
        super().__init__()
        self.split_alpha = split_alpha
        self.query = nn.Embedding(NUM_SLOTS, DIM)
        self.agent_layer, self.map_layer = decoder_layer(), decoder_layer()
        self.bev_layer = decoder_layer() if use_bev else None
        self.norm = nn.LayerNorm(DIM)
        if split_alpha:
            self.alpha_query = nn.Embedding(NUM_SLOTS, DIM)
            self.alpha_agent_layer, self.alpha_map_layer = decoder_layer(), decoder_layer()
            self.alpha_bev_layer = decoder_layer() if use_bev else None
            self.alpha_norm = nn.LayerNorm(DIM)
        self.exist = nn.Linear(DIM, 1)
        self.center = mlp(DIM, DIM, NUM_PATH_POINTS * 2)
        self.half_width = mlp(DIM, DIM, NUM_PATH_POINTS)
        self.alpha = mlp(DIM, DIM, NUM_PATH_POINTS)
        self.valid = mlp(DIM, DIM, NUM_PATH_POINTS)
        for head in (self.center, self.alpha):
            nn.init.normal_(head[-1].weight, std=0.01)
            nn.init.zeros_(head[-1].bias)

    def init_alpha_branch_from_shared(self):
        # 分離した alpha 枝へ従来枝の重みをコピーし、初期の alpha 出力を元と一致させる
        if not self.split_alpha:
            raise RuntimeError('split_alpha=False のモデルには alpha 枝がない')
        pairs = [(self.alpha_query, self.query), (self.alpha_agent_layer, self.agent_layer),
                 (self.alpha_map_layer, self.map_layer), (self.alpha_norm, self.norm)]
        if self.bev_layer is not None:
            pairs.append((self.alpha_bev_layer, self.bev_layer))
        for dst, src in pairs:
            dst.load_state_dict(src.state_dict())

    def embed(self, query, agent_layer, map_layer, bev_layer, norm, agent_tokens, map_tokens, bev_tokens):
        x = query.weight[None].expand(agent_tokens.shape[0], -1, -1)
        x = agent_layer(x, agent_tokens)
        x = map_layer(x, map_tokens)
        if bev_layer is not None:
            x = bev_layer(x, bev_tokens)
        return norm(x)

    def forward(self, agent_tokens, map_tokens, bev_tokens):
        b = agent_tokens.shape[0]
        emb = self.embed(self.query, self.agent_layer, self.map_layer, self.bev_layer, self.norm,
                         agent_tokens, map_tokens, bev_tokens)
        alpha_emb = None
        if self.split_alpha:
            alpha_emb = self.embed(self.alpha_query, self.alpha_agent_layer, self.alpha_map_layer, self.alpha_bev_layer,
                                   self.alpha_norm, agent_tokens, map_tokens, bev_tokens)
        with torch.autocast(device_type=emb.device.type, enabled=False):
            return self.geometry(emb.float(), b, None if alpha_emb is None else alpha_emb.float())

    def geometry(self, emb, b, alpha_emb=None):
        # cumsum・接線正規化・半幅は AMP 下でも fp32
        raw = self.center(emb).view(b, NUM_SLOTS, NUM_PATH_POINTS, 2)
        px = torch.cumsum(PATH_STEP * torch.sigmoid(raw[..., 0] + STEP_BIAS), dim=-1)
        center = torch.stack([px, raw[..., 1]], dim=-1)
        previous = torch.cat([torch.zeros_like(center[:, :, :1]), center[:, :, :-1]], dim=2)
        following = torch.cat([center[:, :, 1:], 2 * center[:, :, -1:] - center[:, :, -2:-1]], dim=2)
        tangent = following - previous
        tangent = tangent / (tangent ** 2).sum(-1, keepdim=True).add(TANGENT_EPS).sqrt()
        normal = torch.stack([-tangent[..., 1], tangent[..., 0]], dim=-1)
        half = (HALF_WIDTH_MIN + nn.functional.softplus(self.half_width(emb) + HALF_WIDTH_BIAS))[..., None] * normal
        alpha = torch.tanh(self.alpha(emb if alpha_emb is None else alpha_emb))
        return {'branch_exist': self.exist(emb)[..., 0], 'corridor_left': center + half,
                'corridor_right': center - half, 'path_valid': self.valid(emb),
                'path_xy': center + alpha[..., None] * half, 'path_alpha': alpha}
