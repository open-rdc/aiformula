import torch
import torch.nn as nn


class ConvStem(nn.Module):
    """画像を 1/8 解像度の特徴マップに落とし、Transformer 用のトークン列に変換する"""

    def __init__(self, in_channels: int, embed_dim: int):
        super(ConvStem, self).__init__()

        self.conv1 = nn.Conv2d(in_channels, 32, kernel_size=3, stride=2, padding=1)
        self.bn1 = nn.BatchNorm2d(32)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1)
        self.bn2 = nn.BatchNorm2d(64)
        self.conv3 = nn.Conv2d(64, embed_dim, kernel_size=3, stride=2, padding=1)
        self.bn3 = nn.BatchNorm2d(embed_dim)

        self.relu = nn.ReLU()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.relu(self.bn1(self.conv1(x)))
        x = self.relu(self.bn2(self.conv2(x)))
        x = self.relu(self.bn3(self.conv3(x)))

        # (B, C, H, W) -> (B, H * W, C)
        x = x.flatten(2).transpose(1, 2)

        return x


class TransformerBlock(nn.Module):
    """Pre-norm の self-attention ブロック。マスクトークンとRGBトークンを同一列で扱うため
    self-attention がそのままモーダル間の cross-attention として働く"""

    def __init__(self, embed_dim: int, num_heads: int, mlp_ratio: int = 4, dropout: float = 0.1):
        super(TransformerBlock, self).__init__()

        self.norm1 = nn.LayerNorm(embed_dim)
        self.attn = nn.MultiheadAttention(embed_dim, num_heads, dropout=dropout, batch_first=True)
        self.norm2 = nn.LayerNorm(embed_dim)
        self.mlp = nn.Sequential(
            nn.Linear(embed_dim, embed_dim * mlp_ratio),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(embed_dim * mlp_ratio, embed_dim),
            nn.Dropout(dropout),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        h = self.norm1(x)
        attn_out, _ = self.attn(h, h, h, need_weights=False)
        x = x + attn_out
        x = x + self.mlp(self.norm2(x))

        return x


class Network(nn.Module):
    """白線マスク画像とRGB画像をそれぞれトークン化し、Transformer で結合して waypoint を回帰する"""

    def __init__(
        self,
        num_waypoints: int = 10,
        image_height: int = 48,
        image_width: int = 64,
        embed_dim: int = 128,
        num_heads: int = 4,
        num_layers: int = 4,
        dropout: float = 0.1,
    ):
        super(Network, self).__init__()

        self.mask_stem = ConvStem(1, embed_dim)
        self.rgb_stem = ConvStem(3, embed_dim)

        # ConvStem は stride 2 を 3 回かけるので解像度は 1/8
        num_patches = (image_height // 8) * (image_width // 8)
        num_tokens = num_patches * 2 + 1

        self.query_token = nn.Parameter(torch.zeros(1, 1, embed_dim))
        self.mask_type_embed = nn.Parameter(torch.zeros(1, 1, embed_dim))
        self.rgb_type_embed = nn.Parameter(torch.zeros(1, 1, embed_dim))
        self.pos_embed = nn.Parameter(torch.zeros(1, num_tokens, embed_dim))
        self.pos_drop = nn.Dropout(dropout)

        self.blocks = nn.ModuleList([
            TransformerBlock(embed_dim, num_heads, 4, dropout) for _ in range(num_layers)
        ])
        self.norm = nn.LayerNorm(embed_dim)

        self.head = nn.Sequential(
            nn.Linear(embed_dim, embed_dim),
            nn.GELU(),
            nn.Linear(embed_dim, num_waypoints * 2),
        )

        self._init_parameters()

    def _init_parameters(self) -> None:
        nn.init.trunc_normal_(self.query_token, std=0.02)
        nn.init.trunc_normal_(self.mask_type_embed, std=0.02)
        nn.init.trunc_normal_(self.rgb_type_embed, std=0.02)
        nn.init.trunc_normal_(self.pos_embed, std=0.02)

    def forward(self, mask: torch.Tensor, rgb: torch.Tensor) -> torch.Tensor:
        batch_size = mask.shape[0]

        mask_tokens = self.mask_stem(mask) + self.mask_type_embed
        rgb_tokens = self.rgb_stem(rgb) + self.rgb_type_embed

        query = self.query_token.expand(batch_size, -1, -1)
        x = torch.cat([query, mask_tokens, rgb_tokens], dim=1)
        x = self.pos_drop(x + self.pos_embed)

        for block in self.blocks:
            x = block(x)

        x = self.norm(x)
        x = self.head(x[:, 0])

        return x


class NetworkEarlyFusion(nn.Module):
    """マスクとRGBを画素のまま 4ch に重ねて 1 本の ConvStem に通す（early fusion）。

    Network (late fusion) との違い:
      * マスクとRGBは同じカメラ・同じリサイズで画素が一対一に対応しているので、
        「この画素は白線で、かつ路面が明るい」といった関係を conv の第1層で直接扱える。
        late fusion では同じ関係を作るのに attention を1往復させる必要がある。
      * トークン数が半分になる (576*2+1 -> 576+1)。attention は O(N^2) なので
        計算量が大きく落ちる。同時に暗記に使える容量も減る。
      * モダリティ種別の embedding (mask_type_embed / rgb_type_embed) が要らなくなる。

    入出力は Network と同じ forward(mask, rgb) にしてある。推論ノードは TorchScript を
    読んで model(mask, rgb) を呼ぶだけなので、どちらを学習しても差し替えるだけで動く。
    """

    def __init__(
        self,
        num_waypoints: int = 10,
        image_height: int = 48,
        image_width: int = 64,
        embed_dim: int = 128,
        num_heads: int = 4,
        num_layers: int = 4,
        dropout: float = 0.1,
        num_past_frames: int = 0,
        num_mask_channels: int = 1,
    ):
        super(NetworkEarlyFusion, self).__init__()

        # マスク(白線 1ch、パイロンを足すと 2ch) + 3ch(現在のRGB) + 3ch x 過去フレーム数
        self.stem = ConvStem(num_mask_channels + 3 + 3 * num_past_frames, embed_dim)

        num_patches = (image_height // 8) * (image_width // 8)
        num_tokens = num_patches + 1

        self.query_token = nn.Parameter(torch.zeros(1, 1, embed_dim))
        self.pos_embed = nn.Parameter(torch.zeros(1, num_tokens, embed_dim))
        self.pos_drop = nn.Dropout(dropout)

        self.blocks = nn.ModuleList([
            TransformerBlock(embed_dim, num_heads, 4, dropout) for _ in range(num_layers)
        ])
        self.norm = nn.LayerNorm(embed_dim)

        self.head = nn.Sequential(
            nn.Linear(embed_dim, embed_dim),
            nn.GELU(),
            nn.Linear(embed_dim, num_waypoints * 2),
        )

        self._init_parameters()

    def _init_parameters(self) -> None:
        nn.init.trunc_normal_(self.query_token, std=0.02)
        nn.init.trunc_normal_(self.pos_embed, std=0.02)

    def forward(self, mask: torch.Tensor, rgb: torch.Tensor) -> torch.Tensor:
        """rgb は (B, 3, H, W) か、過去フレームを重ねた (B, 3*(1+past), H, W)。
        過去フレームは学習データ生成時に**距離**で選んである（時間ではない）"""
        batch_size = mask.shape[0]

        # マスクは 0/1、RGB は [-1, 1] とレンジが違う。同じ conv のカーネルに
        # 束ねて入れるので、レンジを RGB に揃えておく。
        # (late fusion ではマスクに専用の stem と BatchNorm があるので不要だった)
        tokens = self.stem(torch.cat([mask * 2.0 - 1.0, rgb], dim=1))

        query = self.query_token.expand(batch_size, -1, -1)
        x = torch.cat([query, tokens], dim=1)
        x = self.pos_drop(x + self.pos_embed)

        for block in self.blocks:
            x = block(x)

        x = self.norm(x)
        x = self.head(x[:, 0])

        return x


class CrossAttentionBlock(nn.Module):
    """Pre-norm の self-attention → cross-attention (memory を見る) → MLP"""

    def __init__(self, embed_dim: int, num_heads: int, mlp_ratio: int = 4, dropout: float = 0.1):
        super(CrossAttentionBlock, self).__init__()

        self.norm1 = nn.LayerNorm(embed_dim)
        self.self_attn = nn.MultiheadAttention(embed_dim, num_heads, dropout=dropout, batch_first=True)
        self.norm2 = nn.LayerNorm(embed_dim)
        self.norm_mem = nn.LayerNorm(embed_dim)
        self.cross_attn = nn.MultiheadAttention(embed_dim, num_heads, dropout=dropout, batch_first=True)
        self.norm3 = nn.LayerNorm(embed_dim)
        self.mlp = nn.Sequential(
            nn.Linear(embed_dim, embed_dim * mlp_ratio),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(embed_dim * mlp_ratio, embed_dim),
            nn.Dropout(dropout),
        )

    def forward(self, x: torch.Tensor, memory: torch.Tensor) -> torch.Tensor:
        h = self.norm1(x)
        attn_out, _ = self.self_attn(h, h, h, need_weights=False)
        x = x + attn_out
        m = self.norm_mem(memory)
        cross_out, _ = self.cross_attn(self.norm2(x), m, m, need_weights=False)
        x = x + cross_out
        x = x + self.mlp(self.norm3(x))

        return x


class _FrameWise(nn.Module):
    """過去フレームを 1 フレームずつ同じ ConvStem に通す作りの共通部分 (2026-11-01)。

    early fusion は今と過去のフレームを画素のまま重ねてから畳み込むので、車が進んで画面の中で
    位置が変わった物 (近くのパイロンなど) が、同じマスで別の物と混ざる。こちらは各フレームを先に
    同じ重みの ConvStem で特徴 (トークン) にしてから合わせる。

    入力は early と同じ forward(mask, rgb):
      mask (B, 2F, H, W) = フレームごとに [白線, 障害物] (今, -d1 m, -d2 m, ...)
      rgb  (B, 3F, H, W) = フレームごとに 3 ch (今, -d1 m, -d2 m, ...)
    """

    def __init__(self, image_height: int, image_width: int, embed_dim: int, num_frames: int):
        super(_FrameWise, self).__init__()
        self.num_frames = num_frames
        self.num_patches = (image_height // 8) * (image_width // 8)
        # 1 フレーム = 白線 + 障害物 + RGB の 5 ch
        self.stem = ConvStem(5, embed_dim)

    def frame_tokens(self, mask: torch.Tensor, rgb: torch.Tensor) -> torch.Tensor:
        """(B, F, P, D)。全フレームを 1 回でまとめて stem に通す"""
        b, _, h, w = mask.shape
        m = mask.view(b, self.num_frames, 2, h, w)
        c = rgb.view(b, self.num_frames, 3, h, w)
        x = torch.cat([m * 2.0 - 1.0, c], dim=2).view(b * self.num_frames, 5, h, w)
        tokens = self.stem(x)
        return tokens.view(b, self.num_frames, self.num_patches, tokens.shape[-1])


def _head(embed_dim: int, num_waypoints: int) -> nn.Module:
    return nn.Sequential(nn.Linear(embed_dim, embed_dim), nn.GELU(), nn.Linear(embed_dim, num_waypoints * 2))


class NetworkMidFusion(nn.Module):
    """各フレームを同じ ConvStem に通し、1/8 解像度の特徴を同じマスどうしで ch 方向に並べて合わせる。
    トークン数は early と同じ 577"""

    def __init__(self, num_waypoints: int = 10, image_height: int = 48, image_width: int = 64,
                 embed_dim: int = 128, num_heads: int = 4, num_layers: int = 4, dropout: float = 0.1,
                 num_frames: int = 3):
        super(NetworkMidFusion, self).__init__()
        self.frames = _FrameWise(image_height, image_width, embed_dim, num_frames)
        self.fuse = nn.Linear(embed_dim * num_frames, embed_dim)
        self.query_token = nn.Parameter(torch.zeros(1, 1, embed_dim))
        self.pos_embed = nn.Parameter(torch.zeros(1, self.frames.num_patches + 1, embed_dim))
        self.pos_drop = nn.Dropout(dropout)
        self.blocks = nn.ModuleList([TransformerBlock(embed_dim, num_heads, 4, dropout) for _ in range(num_layers)])
        self.norm = nn.LayerNorm(embed_dim)
        self.head = _head(embed_dim, num_waypoints)
        nn.init.trunc_normal_(self.query_token, std=0.02)
        nn.init.trunc_normal_(self.pos_embed, std=0.02)

    def forward(self, mask: torch.Tensor, rgb: torch.Tensor) -> torch.Tensor:
        t = self.frames.frame_tokens(mask, rgb)  # (B, F, P, D)
        b = t.shape[0]
        t = self.fuse(t.permute(0, 2, 1, 3).reshape(b, t.shape[2], -1))  # (B, P, D)
        x = torch.cat([self.query_token.expand(b, -1, -1), t], dim=1)
        x = self.pos_drop(x + self.pos_embed)
        for block in self.blocks:
            x = block(x)
        return self.head(self.norm(x)[:, 0])


class NetworkTokenFusion(nn.Module):
    """各フレームを同じ ConvStem でトークンにし、何フレーム目かの印を足して 1 本の列に並べ、
    Transformer で全フレームのトークンを見比べる。トークン数は 1 + 576 * F"""

    def __init__(self, num_waypoints: int = 10, image_height: int = 48, image_width: int = 64,
                 embed_dim: int = 128, num_heads: int = 4, num_layers: int = 4, dropout: float = 0.1,
                 num_frames: int = 3):
        super(NetworkTokenFusion, self).__init__()
        self.frames = _FrameWise(image_height, image_width, embed_dim, num_frames)
        self.query_token = nn.Parameter(torch.zeros(1, 1, embed_dim))
        self.query_pos = nn.Parameter(torch.zeros(1, 1, embed_dim))
        self.pos_embed = nn.Parameter(torch.zeros(1, 1, self.frames.num_patches, embed_dim))  # 画面の位置
        self.frame_embed = nn.Parameter(torch.zeros(1, num_frames, 1, embed_dim))  # 何フレーム目か
        self.pos_drop = nn.Dropout(dropout)
        self.blocks = nn.ModuleList([TransformerBlock(embed_dim, num_heads, 4, dropout) for _ in range(num_layers)])
        self.norm = nn.LayerNorm(embed_dim)
        self.head = _head(embed_dim, num_waypoints)
        for p in (self.query_token, self.query_pos, self.pos_embed, self.frame_embed):
            nn.init.trunc_normal_(p, std=0.02)

    def forward(self, mask: torch.Tensor, rgb: torch.Tensor) -> torch.Tensor:
        t = self.frames.frame_tokens(mask, rgb) + self.pos_embed + self.frame_embed  # (B, F, P, D)
        b = t.shape[0]
        x = torch.cat([self.query_token.expand(b, -1, -1) + self.query_pos, t.reshape(b, -1, t.shape[-1])], dim=1)
        x = self.pos_drop(x)
        for block in self.blocks:
            x = block(x)
        return self.head(self.norm(x)[:, 0])


class NetworkCrossFusion(nn.Module):
    """各フレームを同じ ConvStem と同じ Transformer (前半 num_layers // 2 段) で別々に処理してから、
    今のフレームのトークン (と答えを集める係) が、過去フレームのトークンを cross-attention で見る (後半)"""

    def __init__(self, num_waypoints: int = 10, image_height: int = 48, image_width: int = 64,
                 embed_dim: int = 128, num_heads: int = 4, num_layers: int = 4, dropout: float = 0.1,
                 num_frames: int = 3):
        super(NetworkCrossFusion, self).__init__()
        self.frames = _FrameWise(image_height, image_width, embed_dim, num_frames)
        self.pos_embed = nn.Parameter(torch.zeros(1, self.frames.num_patches, embed_dim))
        self.frame_embed = nn.Parameter(torch.zeros(1, num_frames, 1, embed_dim))
        self.query_token = nn.Parameter(torch.zeros(1, 1, embed_dim))
        self.pos_drop = nn.Dropout(dropout)
        n_enc = num_layers // 2
        self.encoder = nn.ModuleList([TransformerBlock(embed_dim, num_heads, 4, dropout) for _ in range(n_enc)])
        self.decoder = nn.ModuleList([CrossAttentionBlock(embed_dim, num_heads, 4, dropout)
                                      for _ in range(num_layers - n_enc)])
        self.norm = nn.LayerNorm(embed_dim)
        self.head = _head(embed_dim, num_waypoints)
        for p in (self.pos_embed, self.frame_embed, self.query_token):
            nn.init.trunc_normal_(p, std=0.02)

    def forward(self, mask: torch.Tensor, rgb: torch.Tensor) -> torch.Tensor:
        t = self.frames.frame_tokens(mask, rgb)  # (B, F, P, D)
        b, f, p, d = t.shape
        # 前半: フレームごとに別々に (同じ重みで) self-attention
        x = self.pos_drop(t + self.pos_embed.unsqueeze(1)).reshape(b * f, p, d)
        for block in self.encoder:
            x = block(x)
        x = x.view(b, f, p, d) + self.frame_embed
        current = torch.cat([self.query_token.expand(b, -1, -1), x[:, 0]], dim=1)  # (B, 1 + P, D)
        memory = x[:, 1:].reshape(b, (f - 1) * p, d)  # 過去フレームのトークン
        # 後半: 今のフレームが過去フレームを cross-attention で見る
        for block in self.decoder:
            current = block(current, memory)
        return self.head(self.norm(current)[:, 0])


class GatedQueryCrossAttention(nn.Module):
    """答えを集める係 (クエリトークン 1 個) だけが過去フレームのトークンを cross-attention で見る。
    ゲートを 0 で始める (tanh(0) = 0) ので、学習の始めは過去を見ない単フレームのモデルと同じに振る舞い、
    役に立つ分だけ学習で開いていく"""

    def __init__(self, embed_dim: int, num_heads: int, dropout: float = 0.1):
        super(GatedQueryCrossAttention, self).__init__()
        self.norm_q = nn.LayerNorm(embed_dim)
        self.norm_mem = nn.LayerNorm(embed_dim)
        self.attn = nn.MultiheadAttention(embed_dim, num_heads, dropout=dropout, batch_first=True)
        self.gate = nn.Parameter(torch.zeros(1))

    def forward(self, x: torch.Tensor, memory: torch.Tensor) -> torch.Tensor:
        m = self.norm_mem(memory)
        out, _ = self.attn(self.norm_q(x[:, :1]), m, m, need_weights=False)
        return torch.cat([x[:, :1] + torch.tanh(self.gate) * out, x[:, 1:]], dim=1)


class NetworkCrossLite(nn.Module):
    """cross の軽量版 (調査のまとめ docs/research/temporal_fusion_survey_20261101.md の推奨)。
    今のフレームは単フレームのモデルと同じく ConvStem + Transformer 4 段を通す。過去フレームは同じ ConvStem と
    同じ前半 num_layers // 2 段を別々に通し、何フレーム目かの印を足して記憶にする。
    前半の後と最後に、答えを集める係だけが記憶を cross-attention で見る (ゲートは 0 から)"""

    def __init__(self, num_waypoints: int = 10, image_height: int = 48, image_width: int = 64,
                 embed_dim: int = 128, num_heads: int = 4, num_layers: int = 4, dropout: float = 0.1,
                 num_frames: int = 3):
        super(NetworkCrossLite, self).__init__()
        self.frames = _FrameWise(image_height, image_width, embed_dim, num_frames)
        self.n_enc = num_layers // 2
        self.query_token = nn.Parameter(torch.zeros(1, 1, embed_dim))
        self.pos_embed = nn.Parameter(torch.zeros(1, self.frames.num_patches + 1, embed_dim))
        self.frame_embed = nn.Parameter(torch.zeros(1, num_frames - 1, 1, embed_dim))
        self.pos_drop = nn.Dropout(dropout)
        self.blocks = nn.ModuleList([TransformerBlock(embed_dim, num_heads, 4, dropout) for _ in range(num_layers)])
        self.cross_mid = GatedQueryCrossAttention(embed_dim, num_heads, dropout)
        self.cross_end = GatedQueryCrossAttention(embed_dim, num_heads, dropout)
        self.norm = nn.LayerNorm(embed_dim)
        self.head = _head(embed_dim, num_waypoints)
        for p in (self.query_token, self.pos_embed, self.frame_embed):
            nn.init.trunc_normal_(p, std=0.02)

    def forward(self, mask: torch.Tensor, rgb: torch.Tensor) -> torch.Tensor:
        t = self.frames.frame_tokens(mask, rgb)  # (B, F, P, D)
        b, f, p, d = t.shape
        x = torch.cat([self.query_token.expand(b, -1, -1), t[:, 0]], dim=1)
        x = self.pos_drop(x + self.pos_embed)
        # 過去フレーム: 同じ前半の段を、フレームごとに別々に通す
        past = self.pos_drop(t[:, 1:] + self.pos_embed[:, 1:].unsqueeze(1)).reshape(b * (f - 1), p, d)
        memory = past.reshape(b, (f - 1) * p, d)  # 前半を通した後に作り直す (TorchScript は変数の事前定義が要る)
        for i, block in enumerate(self.blocks):
            if i < self.n_enc:
                past = block(past)
            x = block(x)
            if i == self.n_enc - 1:
                memory = (past.reshape(b, f - 1, p, d) + self.frame_embed).reshape(b, (f - 1) * p, d)
                x = self.cross_mid(x, memory)
        x = self.cross_end(x, memory)
        return self.head(self.norm(x)[:, 0])


def build_network(fusion: str, **kwargs) -> nn.Module:
    """fusion: 'late' = Network (モダリティ別 stem + Transformer 融合)
                'early' = NetworkEarlyFusion (4ch にして 1 本の stem)
                'mid' / 'token' / 'cross' / 'cross_lite' = 過去フレームを 1 フレームずつ同じ stem に通してから合わせる
                (NetworkMidFusion / NetworkTokenFusion / NetworkCrossFusion / NetworkCrossLite)"""
    if fusion == 'late':
        return Network(**kwargs)
    if fusion == 'early':
        return NetworkEarlyFusion(**kwargs)
    if fusion in FRAMEWISE_FUSIONS:
        return FRAMEWISE_FUSIONS[fusion](**kwargs)
    raise ValueError(f"fusion は 'late' / 'early' / 'mid' / 'token' / 'cross' / 'cross_lite': {fusion}")


FRAMEWISE_FUSIONS = {'mid': NetworkMidFusion, 'token': NetworkTokenFusion, 'cross': NetworkCrossFusion,
                     'cross_lite': NetworkCrossLite}
