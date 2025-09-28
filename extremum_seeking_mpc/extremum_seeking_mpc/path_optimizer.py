import numpy as np

from rclpy.node import Node

from .back_propagation import BackPropagation
from .extremum_seeking_controller import ExtremumSeekingController
from .util import ControllerParameters, LowPassFilterParameters


class PathOptimizer:
    def __init__(self, node: Node, control_period: float):
        node.declare_parameter("controller_parameters.seek_gain")
        seek_gain = node.get_parameter("controller_parameters.seek_gain").value

        node.declare_parameter("controller_parameters.seek_amp")
        seek_amp = node.get_parameter("controller_parameters.seek_amp").value

        node.declare_parameter("controller_parameters.curvature_limit")
        curvature_limit = node.get_parameter("controller_parameters.curvature_limit").value

        node.declare_parameter("controller_parameters.feedback_gain")
        feedback_gain = node.get_parameter("controller_parameters.feedback_gain").value

        node.declare_parameter("controller_parameters.sin_period")
        sin_period = node.get_parameter("controller_parameters.sin_period").value

        mpc_params = ControllerParameters(
            seek_gain=seek_gain,
            seek_amp=seek_amp,
            curvature_max=curvature_limit,
            curvature_min=-curvature_limit,
            feedback_gain=feedback_gain,
            sin_period=sin_period
        )

        node.declare_parameter("lowpassfilter.A")
        lowpass_A = node.get_parameter("lowpassfilter.A").value

        node.declare_parameter("lowpassfilter.B")
        lowpass_B = node.get_parameter("lowpassfilter.B").value

        node.declare_parameter("lowpassfilter.C")
        lowpass_C = node.get_parameter("lowpassfilter.C").value

        lowpass_params = LowPassFilterParameters(
            A=lowpass_A, B=lowpass_B, C=lowpass_C
        )

        self.extremum_seeking_controllers = [
            ExtremumSeekingController(mpc_params, lowpass_params, control_period)
            for _ in range(3)
        ]

        self.backpropagation = BackPropagation(node)

    def apply_extremum_seeking_control(self, risk: np.ndarray) -> np.ndarray:
        controllers = self.extremum_seeking_controllers

        # Calculate moving averages
        risk_moving_averages = [
            controller.apply_risk_moving_average(risk) for controller, risk in zip(controllers, risk)
        ]

        # Calculate backpropagation values
        backpropagation_values = {
            "21": self.backpropagation.apply_backpropagation(risk_moving_averages[1], risk_moving_averages[2]),
            "20": self.backpropagation.apply_backpropagation(risk_moving_averages[0], risk_moving_averages[2]),
            "10": self.backpropagation.apply_backpropagation(risk_moving_averages[0], risk_moving_averages[1])
        }

        # Optimize control inputs
        curvatures = [
            controllers[0].optimize_input(
                risk_moving_averages[0], backpropagation_values["20"] + backpropagation_values["10"]),
            controllers[1].optimize_input(
                risk_moving_averages[1], backpropagation_values["21"]),
            controllers[2].optimize_input(
                risk_moving_averages[2], 0)
        ]

        return np.array(curvatures)
