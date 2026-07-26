import argparse

import numpy as np
import torch

from lib.model import PilotNet


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--output-dim', type=int, default=1)
    args = parser.parse_args()

    model = PilotNet(output_dim=args.output_dim)
    model.load_state_dict(torch.load(args.checkpoint, map_location='cpu'))
    model.eval()

    weights = {name: tensor.cpu().numpy() for name, tensor in model.state_dict().items()}
    np.save(args.out, weights, allow_pickle=True)
    print(f'wrote {len(weights)} arrays to {args.out}')


if __name__ == '__main__':
    main()
