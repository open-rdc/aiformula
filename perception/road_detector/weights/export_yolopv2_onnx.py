#!/usr/bin/env python3
"""yolopv2.pt (TorchScript) から白線マスク出力 ll のみを持つ ONNX を書き出す。"""
import argparse
import pathlib

import torch

WEIGHTS_DIR = pathlib.Path(__file__).resolve().parent


class LaneLineOnly(torch.nn.Module):
    """YOLOPv2 の 3 出力のうち ll(白線セグメンテーション)だけを返すラッパ"""

    def __init__(self, model):
        super().__init__()
        self.model = model

    def forward(self, images):
        return self.model(images)[2]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", default=str(WEIGHTS_DIR / "yolopv2.pt"))
    parser.add_argument("--output", default=str(WEIGHTS_DIR / "yolopv2.onnx"))
    parser.add_argument("--height", type=int, default=384, help="nHD なら 384、SVGA なら 416")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--opset", type=int, default=12)
    args = parser.parse_args()

    model = torch.jit.load(args.weights, map_location="cpu")
    model.eval()
    wrapper = LaneLineOnly(model).eval()

    dummy = torch.zeros(1, 3, args.height, args.width)
    with torch.no_grad():
        # ScriptModule を直接 export すると内部トレースが失敗するため先に trace する
        traced = torch.jit.trace(wrapper, dummy)
        torch.onnx.export(
            traced,
            dummy,
            args.output,
            input_names=["images"],
            output_names=["ll"],
            opset_version=args.opset,
            do_constant_folding=True,
        )
    print(f"{args.output} を書き出しました (入力 1x3x{args.height}x{args.width}, 出力 ll)")


if __name__ == "__main__":
    main()
