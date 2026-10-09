import torch
from torch import nn


class Conv(nn.Module):
    def __init__(self, in_channels, out_channels, SiLUivation, k=1, s=1, p=0, g=1):
        super(Conv, self).__init__()
        self.conv = nn.Conv2d(in_channels, out_channels, k, s, p, groups=g, bias=False)
        self.norm = nn.BatchNorm2d(out_channels, eps=0.001, momentum=0.03)
        self.relu = SiLUivation

    def forward(self, x):
        return self.relu(self.norm(self.conv(x)))

    def fuse_forward(self, x):
        return self.relu(self.conv(x))


class Residual(nn.Module):
    def __init__(self, channels, e=0.5):
        super(Residual, self).__init__()
        self.conv1 = Conv(channels, int(channels * e), nn.SiLU(), k=3, p=1)
        self.conv2 = Conv(int(channels * e), channels, nn.SiLU(), k=3, p=1)

    def forward(self, x):
        return x + self.conv2(self.conv1(x))


class C3K(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(C3K, self).__init__()
        self.conv1 = Conv(in_channels, out_channels // 2, nn.SiLU())
        self.conv2 = Conv(in_channels, out_channels // 2, nn.SiLU())
        self.conv3 = Conv(2 * (out_channels // 2), out_channels, nn.SiLU())
        self.res_m = nn.Sequential(
            Residual(out_channels // 2, e=1.0),
            Residual(out_channels // 2, e=1.0),
        )

    def forward(self, x):
        y = self.res_m(self.conv1(x))
        return self.conv3(torch.cat((y, self.conv2(x)), dim=1))


class C3K2(nn.Module):
    def __init__(self, in_channels, out_channels, n, csp, r):
        super(C3K2, self).__init__()
        self.conv1 = Conv(in_channels, 2 * (out_channels // r), nn.SiLU())
        self.conv2 = Conv((2 + n) * (out_channels // r), out_channels, nn.SiLU())

        if csp:
            self.res_m = nn.ModuleList(C3K(out_channels // r, out_channels // r) for _ in range(n))
        else:
            self.res_m = nn.ModuleList(Residual(out_channels // r) for _ in range(n))

    def forward(self, x):
        y = list(self.conv1(x).chunk(2, 1))
        y.extend(m(y[-1]) for m in self.res_m)
        return self.conv2(torch.cat(y, dim=1))


class CTX(nn.Module):
    def __init__(self, in_ch, out_ch, n, csp, r, h, w):
        super(CTX, self).__init__()
        self.grid = (h, w)

        # 大域ベクトル
        self.expand = nn.Linear(in_ch, h * w)

        # コンテキストマップ
        self.ctx0 = nn.Conv2d(1, in_ch // r, 3, 1, 1)
        self.ctx1 = nn.Conv2d(in_ch // r, in_ch, 3, 1, 1)
        self.ctx2 = nn.Conv2d(in_ch, out_ch, 3, 1, 1)

        self.SiLU = nn.SiLU()

    def forward(self, x):
        b, _, h, w = x.size()

        # 空間方向を平均して 1 枚の大域ベクトルにし、粗いグリッドへ展開する
        context = self.SiLU(self.expand(x.mean(dim=(2, 3))))
        context = context.view(b, 1, *self.grid)
        context = nn.functional.interpolate(context, size=(h, w), mode="bilinear", align_corners=False)

        gate = self.SiLU(self.ctx0(context))
        gate = self.SiLU(self.ctx1(gate))

        context = self.ctx2(self.SiLU(gate * x + x))

        return context


class SPPF(nn.Module):
    def __init__(self, in_channels, out_channels, k=5):
        super(SPPF, self).__init__()
        hidden = in_channels // 2
        self.conv1 = Conv(in_channels, hidden, nn.SiLU())
        self.conv2 = Conv(hidden * 4, out_channels, nn.SiLU())
        self.res_m = nn.MaxPool2d(k, stride=1, padding=k // 2)

    def forward(self, x):
        y = [self.conv1(x)]
        y.extend(self.res_m(y[-1]) for _ in range(3))
        return self.conv2(torch.cat(y, dim=1))


class Attention(nn.Module):
    def __init__(self, channels, num_heads):
        super(Attention, self).__init__()
        self.num_heads = num_heads
        self.head_dim = channels // num_heads
        self.key_dim = self.head_dim // 2
        self.scale = self.key_dim ** -0.5

        qkv_dim = channels + self.key_dim * num_heads * 2
        self.qkv = Conv(channels, qkv_dim, nn.Identity())
        self.proj = Conv(channels, channels, nn.Identity())
        self.pe = Conv(channels, channels, nn.Identity(), k=3, p=1, g=channels)

    def forward(self, x):
        b, c, h, w = x.shape
        qkv = self.qkv(x).view(b, self.num_heads, self.key_dim * 2 + self.head_dim, h * w)
        q, k, v = qkv.split([self.key_dim, self.key_dim, self.head_dim], dim=2)

        attn = (q.transpose(-2, -1) @ k) * self.scale
        attn = attn.softmax(dim=-1)

        out = (v @ attn.transpose(-2, -1)).view(b, c, h, w)
        return self.proj(out + self.pe(v.reshape(b, c, h, w)))


class PSABlock(nn.Module):
    def __init__(self, channels, num_heads):
        super(PSABlock, self).__init__()
        self.attn = Attention(channels, num_heads)
        self.ffn = nn.Sequential(
            Conv(channels, channels * 2, nn.SiLU()),
            Conv(channels * 2, channels, nn.Identity()),
        )

    def forward(self, x):
        x = x + self.attn(x)
        return x + self.ffn(x)


class C2PSA(nn.Module):
    def __init__(self, in_channels, out_channels, n=1):
        super(C2PSA, self).__init__()
        self.hidden = in_channels // 2
        self.conv1 = Conv(in_channels, self.hidden * 2, nn.SiLU())
        self.conv2 = Conv(self.hidden * 2, out_channels, nn.SiLU())
        self.res_m = nn.Sequential(
            *(PSABlock(self.hidden, self.hidden // 64 if self.hidden >= 64 else 1) for _ in range(n))
        )

    def forward(self, x):
        a, b = self.conv1(x).split((self.hidden, self.hidden), dim=1)
        return self.conv2(torch.cat((a, self.res_m(b)), dim=1))
