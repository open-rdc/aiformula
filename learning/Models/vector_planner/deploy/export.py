import argparse

import torch

from Models.vector_planner.model.network import ExportWrapper, load_network

INPUT_SHAPE = (1, 3, 384, 640)


def export_onnx(checkpoint, output, opset=17):
    model, _ = load_network(checkpoint)
    if not model.signal_head:
        raise ValueError('信号ヘッド付き（--signal-head で学習した）チェックポイントのみ出力できる')
    wrapper = ExportWrapper(model).eval()
    torch.onnx.export(wrapper, torch.zeros(INPUT_SHAPE), output, opset_version=opset,
                      input_names=['image'], output_names=list(wrapper.output_names))
    return list(wrapper.output_names)


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--checkpoint', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--opset', type=int, default=17)
    args = p.parse_args(argv)
    names = export_onnx(args.checkpoint, args.output, args.opset)
    print(f'{args.output} を出力しました (outputs={names})')


if __name__ == '__main__':
    main()
