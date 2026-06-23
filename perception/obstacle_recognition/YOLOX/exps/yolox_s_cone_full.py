#!/usr/bin/env python3
import os
import torch
from torch import nn
from yolox.exp import Exp as MyExp


class Exp(MyExp):
    def __init__(self):
        super().__init__()
        self.depth = 0.33
        self.width = 0.50
        self.exp_name = os.path.split(os.path.realpath(__file__))[1].split(".")[0]

        self.data_dir = "datasets/cone_full"
        self.train_ann = "instances_train2017.json"
        self.val_ann = "instances_val2017.json"
        self.num_classes = 1

        self.input_size = (360, 640)
        self.test_size = (360, 640)
        self.multiscale_range = 0   # 非正方形のためマルチスケールは固定(安全)

        # ---- スケジュール: 300エポック ----
        self.max_epoch = 300
        self.no_aug_epochs = 15
        self.warmup_epochs = 5
        self.data_num_workers = 4
        self.eval_interval = 10
        self.print_interval = 20

        # ---- AdamW 用ハイパラ(SGDより大幅に低いLR) ----
        # peak_lr = basic_lr_per_img * batch_size。batch16なら 2.5e-5*16 = 4e-4
        self.basic_lr_per_img = 2.5e-5
        self.warmup_lr = 0.0
        self.min_lr_ratio = 0.05
        self.weight_decay = 0.01     # AdamW(decoupled)向けに既定0.0005から引き上げ

        # 推論既定
        self.test_conf = 0.25
        self.nmsthre = 0.45

    # SGD既定を AdamW に置き換え(YOLOXのパラメータ分割を踏襲)
    def get_optimizer(self, batch_size):
        if "optimizer" not in self.__dict__:
            lr = self.warmup_lr if self.warmup_epochs > 0 else self.basic_lr_per_img * batch_size
            pg0, pg1, pg2 = [], [], []  # bn重み(wd無), conv重み(wd有), bias(wd無)
            for k, v in self.model.named_modules():
                if hasattr(v, "bias") and isinstance(v.bias, nn.Parameter):
                    pg2.append(v.bias)
                if isinstance(v, nn.BatchNorm2d) or "bn" in k:
                    pg0.append(v.weight)
                elif hasattr(v, "weight") and isinstance(v.weight, nn.Parameter):
                    pg1.append(v.weight)
            optimizer = torch.optim.AdamW(pg0, lr=lr, betas=(0.9, 0.999), weight_decay=0.0)
            optimizer.add_param_group({"params": pg1, "weight_decay": self.weight_decay})
            optimizer.add_param_group({"params": pg2})
            self.optimizer = optimizer
        return self.optimizer
