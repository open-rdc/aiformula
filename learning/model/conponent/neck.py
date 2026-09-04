import torch
from torch import nn

from model.conponent.common_layer import C3K2


class Neck(nn.Module):

    def __init__(self, width, depth, csp):
        super(Neck, self).__init__()
        self.up = nn.Upsample(scale_factor=2, mode="nearest")

        self.h1 = C3K2(width[4] + width[5], width[4], depth[0], csp[0], r=2)
        self.h2 = C3K2(width[4] + width[4], width[3], depth[0], csp[0], r=2)

    def forward(self, x):
        p2, p3, p4, p5 = x
        p4 = self.h1(torch.cat((self.up(p5), p4), dim=1))
        p3 = self.h2(torch.cat((self.up(p4), p3), dim=1))

        return p2, p3
