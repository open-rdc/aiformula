import torch
from torch import nn
from torch.nn.utils.fusion import fuse_conv_bn_eval

from Models.model_components.vision_planner.vision_planner_backbone import Backbone
from Models.model_components.vision_planner.vision_planner_neck import Neck
from Models.model_components.vision_planner.vision_planner_head import Head
from Models.model_components.common_layers import Conv

# (straight / left / right）
NUM_SLOTS = 3


class VisionPlannerNetwork:
    DYNAMIC_WEIGHTING = {
        "n": {"width": [3, 16, 32, 64, 128, 256], "depth": [1, 1, 1, 1], "csp": [False, True]},
        "s": {"width": [3, 32, 64, 128, 256, 512], "depth": [1, 1, 1, 1], "csp": [False, True]},
        "m": {"width": [3, 64, 128, 256, 512, 512], "depth": [1, 1, 1, 1], "csp": [True, True]},
        "l": {"width": [3, 64, 128, 256, 512, 512], "depth": [2, 2, 2, 2], "csp": [True, True]},
        "x": {"width": [3, 96, 192, 384, 768, 768], "depth": [2, 2, 2, 2], "csp": [True, True]},
    }

    def build_model(self, version):
        return Network(version)

    def load_model(self, version, checkpoint_path):
        checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
        # "head" キーが無いチェックポイントは A/B 切り替え導入前(B案=dist)の資産なので
        # "dist" にフォールバックする。既定を argmax にしても既存資産が読み戻せるようにするため
        head_mode = checkpoint.get("head", "dist")
        model = Network(version, head_mode=head_mode, backbone=checkpoint.get("backbone", "ctx"))
        model.load_state_dict(checkpoint["model_state_dict"])
        return model


class Network(nn.Module):
    def __init__(self, version, head_mode="argmax", backbone="ctx"):
        super(Network, self).__init__()
        weighting = VisionPlannerNetwork.DYNAMIC_WEIGHTING[version]
        width, depth, csp = weighting["width"], weighting["depth"], weighting["csp"]
        self.backbone = Backbone(width, depth, csp, block=backbone)
        self.neck = Neck(width, depth, csp)
        self.head = Head(width[3], NUM_SLOTS, mode=head_mode)

    def forward(self, x):
        return self.head(self.neck(self.backbone(x)))

    def fuse(self):
        for m in self.modules():
            if type(m) is Conv and hasattr(m, "norm"):
                m.conv = fuse_conv_bn_eval(m.conv, m.norm)
                m.forward = m.fuse_forward
                delattr(m, "norm")
        return self
