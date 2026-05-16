#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

import cv2
import torch
import yaml
from torchvision import transforms

from e2e_planner.util.preprocessing import MODEL_INPUT_SIZE, center_square_crop


class TopomapGenerator:
    COMMAND_TO_ACTION = {
        0: 'roadside',
        1: 'straight',
        2: 'left',
        3: 'right',
    }
    SAVED_STEP = 5

    def __init__(self, dataset_path):
        self.dataset_root = Path(dataset_path)
        self.image_dir = self.dataset_root / 'images'
        self.command_dir = self.dataset_root / 'commands'

        script_dir = Path(__file__).resolve().parent
        self.package_root = script_dir.parent
        self.topomap_dir = self.package_root / 'config' / 'topomap'
        self.topomap_images_dir = self.topomap_dir / 'images'
        self.topomap_yaml_path = self.topomap_dir / 'topomap.yaml'

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = self.load_model()
        self.placenet_transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize(MODEL_INPUT_SIZE, antialias=True),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])

    def load_model(self):
        weight_path = self.package_root / 'weights' / 'placenet.pt'
        model = torch.jit.load(str(weight_path), map_location=self.device)
        model.eval()
        return model

    def _prepare_directories(self):
        self.topomap_images_dir.mkdir(parents=True, exist_ok=True)

    def _load_command(self, image_path):
        command_path = self.command_dir / f'{image_path.stem}.csv'
        if not command_path.exists():
            return 1

        with command_path.open('r', newline='') as f:
            return int(float(next(csv.reader(f))[0]))

    def _center_crop(self, image_path):
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f'Failed to read image: {image_path}')
        return center_square_crop(image)

    def extract_feature(self, image):
        image_tensor = self.placenet_transform(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)).unsqueeze(0)
        image_tensor = image_tensor.to(self.device, dtype=torch.float32)

        with torch.no_grad():
            output = self.model(image_tensor)

        return output.squeeze(0).flatten().tolist()

    def build_nodes(self):
        image_paths = sorted(self.image_dir.glob('*.png'))
        nodes = []

        for idx, image_path in enumerate(image_paths[::self.SAVED_STEP]):
            command = self._load_command(image_path)
            if command not in self.COMMAND_TO_ACTION:
                raise ValueError(f'Unsupported command value: {command}')

            # センタークロップ（288x288）を特徴抽出と保存用画像の両方のベースにする
            cropped_image = self._center_crop(image_path)

            # 保存用は cv2.resize で 85x85 に縮小
            save_image = cv2.resize(cropped_image, MODEL_INPUT_SIZE, interpolation=cv2.INTER_AREA)
            output_image_name = f'img{idx + 1:05d}.png'
            output_image_path = self.topomap_images_dir / output_image_name
            cv2.imwrite(str(output_image_path), save_image)

            # 特徴抽出は 288x288 のまま渡し、transforms.Resize で縮小する（推論時と同じ経路）
            nodes.append({
                'id': idx,
                'image': output_image_name,
                'feature': self.extract_feature(cropped_image),
                'action': self.COMMAND_TO_ACTION[command],
            })

        if not nodes:
            raise ValueError(f'No images found in dataset: {self.image_dir}')

        for idx, node in enumerate(nodes):
            target = idx + 1 if idx + 1 < len(nodes) else idx
            node['edges'] = [{'target': target, 'action': node.pop('action')}]

        return nodes

    def generate(self):
        self._prepare_directories()
        topomap = {'nodes': self.build_nodes()}

        with self.topomap_yaml_path.open('w', encoding='utf-8') as f:
            yaml.safe_dump(topomap, f, sort_keys=False, allow_unicode=False)


def main():
    if len(sys.argv) != 2:
        print('Usage: python3 create_topomap.py <dataset_path>')
        sys.exit(1)

    dataset_path = Path(sys.argv[1])
    if not dataset_path.exists():
        print(f'Dataset path does not exist: {dataset_path}')
        sys.exit(1)

    topomap_generator = TopomapGenerator(dataset_path)
    topomap_generator.generate()


if __name__ == '__main__':
    main()
