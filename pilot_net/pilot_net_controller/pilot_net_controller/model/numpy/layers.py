import numpy as np


def conv2d(x: np.ndarray, weight: np.ndarray, bias: np.ndarray, stride: int) -> np.ndarray:
    batch, in_channels, in_height, in_width = x.shape
    out_channels, _, kernel_height, kernel_width = weight.shape
    out_height = (in_height - kernel_height) // stride + 1
    out_width = (in_width - kernel_width) // stride + 1

    windows = np.lib.stride_tricks.sliding_window_view(x, (kernel_height, kernel_width), axis=(2, 3))
    windows = windows[:, :, ::stride, ::stride, :, :]
    windows = windows.reshape(batch, in_channels, out_height, out_width, kernel_height * kernel_width)
    windows = windows.transpose(0, 2, 3, 1, 4).reshape(
        batch, out_height, out_width, in_channels * kernel_height * kernel_width)

    weight_matrix = weight.reshape(out_channels, in_channels * kernel_height * kernel_width)
    out = windows @ weight_matrix.T + bias
    return out.transpose(0, 3, 1, 2)


def linear(x: np.ndarray, weight: np.ndarray, bias: np.ndarray) -> np.ndarray:
    return x @ weight.T + bias


def relu(x: np.ndarray) -> np.ndarray:
    return np.maximum(x, 0.0)


def tanh(x: np.ndarray) -> np.ndarray:
    return np.tanh(x)


def flatten(x: np.ndarray) -> np.ndarray:
    return x.reshape(x.shape[0], -1)
