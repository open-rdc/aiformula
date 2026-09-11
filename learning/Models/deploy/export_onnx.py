import argparse

import numpy as np
import onnxruntime
import torch

from Models.model_components.vision_planner.vision_planner_network import VisionPlannerNetwork

INPUT_SHAPE = (1, 3, 384, 640)
OUTPUT_NAMES = ['exist', 'valid', 'position']


def export_onnx(weights_path, output_path, opset=11):
    checkpoint = torch.load(weights_path, map_location='cpu', weights_only=True)
    head = checkpoint.get('head', 'dist')
    if head != 'argmax':
        raise ValueError(
            f'head={head!r} は非対応です。argmax のみ対応します'
            '（dist は decode_positions の窓 softmax を C++ に二重実装する必要があるため）')

    version = checkpoint['version']
    model = VisionPlannerNetwork().load_model(version, weights_path).fuse().eval()

    torch.manual_seed(0)
    dummy = torch.rand(INPUT_SHAPE)
    with torch.no_grad():
        reference = model(dummy)

    torch.onnx.export(model, dummy, output_path,
                      input_names=['input'], output_names=OUTPUT_NAMES,
                      opset_version=opset, do_constant_folding=True)

    session = onnxruntime.InferenceSession(output_path, providers=['CPUExecutionProvider'])
    actual = session.run(None, {'input': dummy.numpy()})
    max_abs_diff = max(float(np.abs(a - b.numpy()).max()) for a, b in zip(actual, reference))

    return {'max_abs_diff': max_abs_diff, 'version': version,
            'backbone': checkpoint.get('backbone', 'ctx')}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--opset', type=int, default=11)
    args = parser.parse_args()

    report = export_onnx(args.weights, args.output, args.opset)
    print(f'{args.output} を出力しました '
          f'(version={report["version"]}, backbone={report["backbone"]}, '
          f'PyTorch との最大絶対誤差={report["max_abs_diff"]:.3e})')


if __name__ == '__main__':
    main()
