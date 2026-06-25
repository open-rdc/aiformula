import torch
import os
import torch.nn as nn
from torchvision import models, transforms
from pathlib import Path
from PIL import Image
import numpy as np


class ImageFeatureExtractor(nn.Module):
    def __init__(self):
        super().__init__()
        self.device = torch.device('cuda')

        backbone = models.efficientnet_b0(weights=None)

        weights_path = Path(__file__).parent.parent / 'weights' / 'efficientnet_b0_rwightman-3dd342df.pth'
        if not weights_path.exists():
            raise FileNotFoundError(f"weights not found: {weights_path}")
        state_dict = torch.load(str(weights_path), map_location='cpu')
        backbone.load_state_dict(state_dict)

        self.features = backbone.features.to(self.device)
        self.pool = backbone.avgpool.to(self.device)

        for param in self.features.parameters():
            param.requires_grad = False

        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225]),
        ])

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.features(x)
        x = self.pool(x)
        return x.flatten(start_dim=1)

    def extract(self, image: np.ndarray) -> np.ndarray:
        pil = Image.fromarray(image)
        tensor = self.transform(pil).unsqueeze(0).to(self.device)
        with torch.no_grad():
            features = self.forward(tensor)
        return features.squeeze(0).cpu().numpy()
