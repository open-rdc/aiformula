import numpy as np

from .numpy.layers import conv2d, flatten, linear, relu, tanh


class PilotNetNp:
    def __init__(self, weights_path: str):
        self.weights = np.load(weights_path, allow_pickle=True).item()

    def forward(self, x: np.ndarray) -> np.ndarray:
        x = relu(conv2d(x, self.weights['conv1.weight'], self.weights['conv1.bias'], stride=2))
        x = relu(conv2d(x, self.weights['conv2.weight'], self.weights['conv2.bias'], stride=2))
        x = relu(conv2d(x, self.weights['conv3.weight'], self.weights['conv3.bias'], stride=2))
        x = relu(conv2d(x, self.weights['conv4.weight'], self.weights['conv4.bias'], stride=1))
        x = relu(conv2d(x, self.weights['conv5.weight'], self.weights['conv5.bias'], stride=1))
        x = flatten(x)
        x = relu(linear(x, self.weights['fc1.weight'], self.weights['fc1.bias']))
        x = relu(linear(x, self.weights['fc2.weight'], self.weights['fc2.bias']))
        x = relu(linear(x, self.weights['fc3.weight'], self.weights['fc3.bias']))
        x = tanh(linear(x, self.weights['fc4.weight'], self.weights['fc4.bias']))
        return x
