"""学習済み重みを val で評価して指標を JSON に出す。

比較実験で複数の重みを同一の val に当てるために使う。学習中の validate と同じ
PathMetrics を通すので、学習ログの値と直接比較できる。
"""
import argparse
import json

import torch
from torch.utils.data import DataLoader

from Models.data_utils.vision_planner.load_data_vision_planner import PathDataset, find_images
from Models.model_components.vision_planner.vision_planner_network import VisionPlannerNetwork
from Models.training.vision_planner_trainer import NUM_SLOTS, device_type_of, validate


def evaluate(weights, dataset, split='val', device='cuda', batch_size=16, workers=8,
             input_height=384, input_width=640):
    paths = find_images(dataset, split)
    if not paths:
        raise ValueError(f'{split} の画像が見つからない: {dataset}')
    loader = DataLoader(
        PathDataset(paths, NUM_SLOTS, input_height // 8, (input_height, input_width)),
        batch_size=batch_size, shuffle=False, num_workers=workers,
        pin_memory=device_type_of(device) == 'cuda')

    checkpoint = torch.load(weights, map_location='cpu', weights_only=True)
    model = VisionPlannerNetwork().load_model(checkpoint['version'], weights).to(device)
    with torch.no_grad():
        result, score = validate(model, loader, NUM_SLOTS, device, input_width)
    result['score'] = score
    result['num_images'] = len(paths)
    result['weights'] = weights
    result['epoch'] = checkpoint.get('epoch')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', required=True)
    parser.add_argument('--dataset', required=True)
    parser.add_argument('--split', default='val')
    parser.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    parser.add_argument('--batch-size', type=int, default=16)
    parser.add_argument('--workers', type=int, default=8)
    parser.add_argument('--out', default=None)
    args = parser.parse_args(argv)

    result = evaluate(args.weights, args.dataset, args.split, args.device,
                      args.batch_size, args.workers)
    text = json.dumps(result, indent=2, ensure_ascii=False)
    print(text)
    if args.out:
        with open(args.out, 'w') as f:
            f.write(text)


if __name__ == '__main__':
    main()
