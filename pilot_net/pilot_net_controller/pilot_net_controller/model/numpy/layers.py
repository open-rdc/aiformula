import numpy as np


def conv2d(x: np.ndarray, weight: np.ndarray, bias: np.ndarray, stride: int) -> np.ndarray:
    batch, in_channels, in_height, in_width = x.shape
    out_channels, _, kernel_height, kernel_width = weight.shape
    out_height = (in_height - kernel_height) // stride + 1
    out_width = (in_width - kernel_width) // stride + 1

    columns = np.empty(
        (batch, in_channels, kernel_height, kernel_width, out_height, out_width), dtype=x.dtype)
    for i in range(kernel_height):
        for j in range(kernel_width):
            columns[:, :, i, j] = x[
                :, :,
                i:i + stride * out_height:stride,
                j:j + stride * out_width:stride]

    weight_matrix = weight.reshape(out_channels, in_channels * kernel_height * kernel_width)
    out = weight_matrix @ columns.reshape(batch, -1, out_height * out_width) + bias[:, np.newaxis]
    return out.reshape(batch, out_channels, out_height, out_width)


def linear(x: np.ndarray, weight: np.ndarray, bias: np.ndarray) -> np.ndarray:
    return x @ weight.T + bias


def relu(x: np.ndarray) -> np.ndarray:
    return np.maximum(x, 0.0)


def tanh(x: np.ndarray) -> np.ndarray:
    return np.tanh(x)


def flatten(x: np.ndarray) -> np.ndarray:
    return x.reshape(x.shape[0], -1)
