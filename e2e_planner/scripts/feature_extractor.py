import torch
import torch.nn as nn
from torchvision import models, transforms
from PIL import Image
import numpy as np


class ImageFeatureExtractor(nn.Module):
    def __init__(self):
        super().__init__()
        self.device = torch.device('cuda')

        weights = models.EfficientNet_B0_Weights.IMAGENET1K_V1
        backbone = models.efficientnet_b0(weights=weights)

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
