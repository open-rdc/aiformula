from torch import nn

from Models.model_components.common_layers import Conv, C3K2, SPPF, C2PSA, CTX

BACKBONE_BLOCKS = ('ctx', 'c3k2')


class Backbone(nn.Module):

    def __init__(self, width, depth, csp, block='ctx'):
        super(Backbone, self).__init__()
        if block not in BACKBONE_BLOCKS:
            raise ValueError(f'未知の backbone: {block!r}（有効値: {BACKBONE_BLOCKS}）')

        def stage(in_ch, out_ch, n, use_csp, r, grid):
            if block == 'ctx':
                return CTX(in_ch, out_ch, n, use_csp, r=r, h=grid, w=grid)
            return C3K2(in_ch, out_ch, n, use_csp, r=r)

        # p1
        self.p1 = Conv(width[0], width[1], nn.SiLU(), k=3, s=2, p=1)

        # p2
        self.p2 = nn.Sequential(
            Conv(width[1], width[2], nn.SiLU(), k=3, s=2, p=1),
            stage(width[2], width[3], depth[0], csp[0], 4, 16),
        )
        # p3
        self.p3 = nn.Sequential(
            Conv(width[3], width[3], nn.SiLU(), k=3, s=2, p=1),
            stage(width[3], width[4], depth[1], csp[0], 4, 8),
        )
        # p4
        self.p4 = nn.Sequential(
            Conv(width[4], width[4], nn.SiLU(), k=3, s=2, p=1),
            stage(width[4], width[4], depth[2], csp[1], 2, 4),
        )
        # p5
        self.p5 = nn.Sequential(
            Conv(width[4], width[5], nn.SiLU(), k=3, s=2, p=1),
            stage(width[5], width[5], depth[3], csp[1], 2, 2),
            SPPF(width[5], width[5]),
            C2PSA(width[5], width[5]),
        )

    def forward(self, x):
        p1 = self.p1(x)
        p2 = self.p2(p1)
        p3 = self.p3(p2)
        p4 = self.p4(p3)
        p5 = self.p5(p4)

        return p2, p3, p4, p5
