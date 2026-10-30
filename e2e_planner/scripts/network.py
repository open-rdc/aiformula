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
    ):
        super(NetworkEarlyFusion, self).__init__()

        # 1ch(マスク) + 3ch(現在のRGB) + 3ch x 過去フレーム数
        self.stem = ConvStem(4 + 3 * num_past_frames, embed_dim)

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


def build_network(fusion: str, **kwargs) -> nn.Module:
    """fusion: 'late' = Network (モダリティ別 stem + Transformer 融合)
                'early' = NetworkEarlyFusion (4ch にして 1 本の stem)"""
    if fusion == 'late':
        return Network(**kwargs)
    if fusion == 'early':
        return NetworkEarlyFusion(**kwargs)
    raise ValueError(f"fusion は 'late' か 'early': {fusion}")
