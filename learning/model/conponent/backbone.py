from torch import nn

from model.conponent.common_layer import Conv, C3K2, SPPF, C2PSA, CTX


class Backbone(nn.Module):

    def __init__(self, width, depth, csp):
        super(Backbone, self).__init__()

        # p1
        self.p1 = Conv(width[0], width[1], nn.SiLU(), k=3, s=2, p=1)

        # p2
        self.p2 = nn.Sequential(
            Conv(width[1], width[2], nn.SiLU(), k=3, s=2, p=1),
            CTX(width[2], width[3], depth[0], csp[0], r=4, h=16, w=16),
        )
        # p3
        self.p3 = nn.Sequential(
            Conv(width[3], width[3], nn.SiLU(), k=3, s=2, p=1),
            CTX(width[3], width[4], depth[1], csp[0], r=4, h=8, w=8),
        )
        # p4
        self.p4 = nn.Sequential(
            Conv(width[4], width[4], nn.SiLU(), k=3, s=2, p=1),
            CTX(width[4], width[4], depth[2], csp[1], r=2, h=4, w=4),
        )
        # p5
        self.p5 = nn.Sequential(
            Conv(width[4], width[5], nn.SiLU(), k=3, s=2, p=1),
            CTX(width[5], width[5], depth[3], csp[1], r=2, h=2, w=2),
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
