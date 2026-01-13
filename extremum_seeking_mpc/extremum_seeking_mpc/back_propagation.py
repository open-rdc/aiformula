import numpy as np
from scipy.interpolate import interp1d

from rclpy.node import Node


class BackPropagation:
    def __init__(self, node: Node):
        self.init_parameters(node)
        self.bpgain_function_positive = interp1d(self.gain_function_u, self.gain_function_positive_y)
        self.bpgain_function_negative = interp1d(self.gain_function_u, self.gain_function_negative_y)

    def init_parameters(self, node: Node):
        node.declare_parameter("backpropagation.gain_function_u")
        self.gain_function_u = node.get_parameter("backpropagation.gain_function_u").value

        node.declare_parameter("backpropagation.gain_function_positive_y")
        self.gain_function_positive_y = node.get_parameter("backpropagation.gain_function_positive_y").value

        node.declare_parameter("backpropagation.gain_function_negative_y")
        self.gain_function_negative_y = node.get_parameter("backpropagation.gain_function_negative_y").value

    def apply_backpropagation(self, forward_risk_in: float, backward_risk_in: float) -> float:
        forward_risk_in = np.clip(forward_risk_in, -1., 1.)
        if backward_risk_in < 0.0:
            propagation_gain = self.bpgain_function_negative(forward_risk_in)
        else:
            propagation_gain = self.bpgain_function_positive(forward_risk_in)

        return backward_risk_in * propagation_gain
