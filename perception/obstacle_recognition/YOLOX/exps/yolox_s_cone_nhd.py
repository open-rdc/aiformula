#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# 本番カメラ nHD(640x360) 向け再学習。YOLOXは入力が32の倍数必須のため
# input_size=(H352, W640)(16:9に最も近い32の倍数)。データはcone_full(統合)。
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

        # ---- 本番nHD(640x360)向け: 32の倍数で16:9に最も近い (H352,W640) ----
        self.input_size = (352, 640)
        self.test_size = (352, 640)
        self.multiscale_range = 0   # 非正方形のため固定

        # ---- スケジュール(前回踏襲: 300ep) ----
        self.max_epoch = 300
        self.no_aug_epochs = 15
        self.warmup_epochs = 5
        self.data_num_workers = 4
        self.eval_interval = 10
        self.print_interval = 20

        # ---- AdamW(前回踏襲) ----
        self.basic_lr_per_img = 2.5e-5      # peak=basic*batch (b16 -> 4e-4)
        self.warmup_lr = 0.0
        self.min_lr_ratio = 0.05
        self.weight_decay = 0.01

        self.test_conf = 0.25
        self.nmsthre = 0.45

    def get_optimizer(self, batch_size):
        if "optimizer" not in self.__dict__:
            lr = self.warmup_lr if self.warmup_epochs > 0 else self.basic_lr_per_img * batch_size
            pg0, pg1, pg2 = [], [], []
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
