from dataclasses import dataclass
from functools import partial
import numpy as np
from scipy.interpolate import interp1d
from scipy.stats import multivariate_normal
from typing import Tuple

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from .util import Side


class RoadRiskCalculator:
    def __init__(self, node: Node, buffer_size: int):
        self.init_parameters(node)
        self.init_connections(node, buffer_size)

        self.point_length = np.zeros((Side.NUM_SIDES))
        self.road_offset = np.zeros(Side.NUM_SIDES)
        self.road_thetas = np.zeros((Side.NUM_SIDES, self.num_weight_function, self.num_weight_function_coefficients))

        weight_u = self.road_weight_u
        weight_y = np.zeros((self.num_weight_function, len(weight_u)))

        weight_indices = [
            slice(None, 2),
            2,
            3,
            4,
            slice(5, None)
        ]

        self.Weight_functions = []

        for idx, weight_idx in enumerate(weight_indices):
            weight_y[idx, weight_idx] = 1.0
            interpolated_function = interp1d(weight_u, weight_y[idx, :])
            self.Weight_functions.append(interpolated_function)

        self.identification_gain_matrix = self.identification_gain * np.eye(self.num_weight_function_coefficients)
        self.forget_vector = np.array(self.forget_vector)
        self.theta_base = np.zeros(self.num_weight_function_coefficients)
        self.thetas = np.zeros((self.num_weight_function, self.num_weight_function_coefficients))
        self.dthetas = np.zeros((self.num_weight_function, self.num_weight_function_coefficients))

    def init_parameters(self, node: Node):
        node.declare_parameter("road_risk_potential.left_gradient")
        self.road_risk_left_gradient = node.get_parameter("road_risk_potential.left_gradient").value

        node.declare_parameter("road_risk_potential.right_gradient")
        self.road_risk_right_gradient = node.get_parameter("road_risk_potential.right_gradient").value

        node.declare_parameter("road_risk_potential.margin")
        self.road_risk_margin = node.get_parameter("road_risk_potential.margin").value

        node.declare_parameter("road_risk_potential.offset")
        self.road_risk_offset = node.get_parameter("road_risk_potential.offset").value

        node.declare_parameter("road_risk_potential.gain")
        self.road_risk_gain = node.get_parameter("road_risk_potential.gain").value

        node.declare_parameter("road_weight.u")
        self.road_weight_u = node.get_parameter("road_weight.u").value

        node.declare_parameter("road_weight.num_function")
        self.num_weight_function = node.get_parameter("road_weight.num_function").value

        node.declare_parameter("road_weight.num_function_coefficients")
        self.num_weight_function_coefficients = node.get_parameter("road_weight.num_function_coefficients").value

        node.declare_parameter("road_identification.max_point_length")
        self.max_point_length = node.get_parameter("road_identification.max_point_length").value

        node.declare_parameter("road_identification.identification_gain")
        self.identification_gain = node.get_parameter("road_identification.identification_gain").value

        node.declare_parameter("road_identification.forget_vector")
        self.forget_vector = node.get_parameter("road_identification.forget_vector").value

        node.declare_parameter("road_identification.road_parameter_limit")
        self.road_parameter_limit = node.get_parameter("road_identification.road_parameter_limit").value

        node.declare_parameter("road_benefit_function.scale")
        self.benefit_scale = node.get_parameter("road_benefit_function.scale").value

        node.declare_parameter("road_benefit_function.covariance")
        self.benefit_covariance = node.get_parameter("road_benefit_function.covariance").value

    def init_connections(self, node: Node, buffer_size):
        self.left_lane_line_sub = node.create_subscription(
            PointCloud2, 'sub_road_l', partial(self.lane_line_callback, side=Side.LEFT), buffer_size)
        self.right_lane_line_sub = node.create_subscription(
            PointCloud2, 'sub_road_r', partial(self.lane_line_callback, side=Side.RIGHT), buffer_size)

    def lane_line_callback(self, msg_pointcloud2: PointCloud2, side: Side) -> None:
        # Read points as structured array
        structured_points = pc2.read_points(msg_pointcloud2, field_names=['x', 'y'], skip_nans=True)
        points_list = list(structured_points)

        if len(points_list) < 2:
            return

        # Convert structured array to regular 2D numpy array
        lane_points = np.array([(point[0], point[1]) for point in points_list])

        self.point_length[side], self.road_offset[side], self.road_thetas[side] = self.line_identification(lane_points)

    def line_identification(self, lane_points: np.ndarray) -> Tuple[float, float, np.ndarray]:
        x_coords, y_coords = lane_points[:, 0], lane_points[:, 1]
        x_min = min(x_coords)
        x_range = max(x_coords) - x_min
        normalized_x = (x_coords - x_min) / x_range

        x_vectors = np.column_stack([
            normalized_x ** 2,
            normalized_x,
            np.ones_like(normalized_x)
        ])

        weights_list = np.array([f(normalized_x) for f in self.Weight_functions]).T

        # Calculate the road function parameters(thetas) using the fixed gain method (sequential identification)
        for x_vector, weights, y_coord in zip(x_vectors, weights_list, y_coords):
            adaptive_gain_numerator = self.identification_gain_matrix @ x_vector
            zp = x_vector @ self.identification_gain_matrix
            adaptive_gain_denominator = 1. + np.dot(zp, x_vector)
            adaptive_gain = adaptive_gain_numerator / adaptive_gain_denominator
            weighted_y_hats = self.thetas @ x_vector * weights
            y_error = y_coord - weighted_y_hats.sum()
            weighted_y_errors = np.outer((y_error * weights), adaptive_gain)
            self.thetas, self.dthetas = self.identify_function_coefficients(weighted_y_errors, self.dthetas)

        return x_range, x_min, self.thetas

    def identify_function_coefficients(self, ddtheta: np.ndarray, dtheta_memory: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        dtheta = ddtheta + (dtheta_memory * self.forget_vector)
        dtheta[:, 0] = np.clip(dtheta[:, 0], -self.road_parameter_limit, self.road_parameter_limit)
        theta = dtheta + np.array(self.theta_base)

        return theta, dtheta

    def estimate_yhat(self, x_u: float, x_range: float, x_min: float, thetas: np.ndarray) -> float:
        normalized_x = (x_u - x_min) / x_range
        x_vectors = np.column_stack([
            normalized_x ** 2,
            normalized_x,
            np.ones_like(normalized_x)
        ])

        weights = [weight_function(normalized_x) for weight_function in self.Weight_functions]
        y_hats = [np.dot(theta, x_vectors.T) * weight for theta, weight in zip(thetas, weights)]

        return sum(y_hats)

    def compute_road_risk(self, seek_positions: np.ndarray, side: Side) -> Tuple[np.ndarray, float]:
        num_positions = len(seek_positions)
        num_seek_position = len(seek_positions[0][0])
        risks = np.zeros((num_positions, num_seek_position))
        y_hat = 0.

        for idx, seek_position in enumerate(seek_positions):
            num_seek_position = len(seek_position[0])
            seek_x = seek_position[0][2]  # center of seek_points
            seek_y_positions = seek_position[1]
            risk = np.zeros(num_seek_position)

            if not self.road_offset[side] < seek_x < self.road_offset[side] + self.point_length[side]:
                risks[idx] = risk
                continue

            y_hat = self.estimate_yhat(seek_x, self.point_length[side], self.road_offset[side], self.road_thetas[side])

            risk = self.get_road_risk_value(seek_y_positions, y_hat, side)
            risks[idx] = risk

        return risks, y_hat

    def get_road_risk_value(self, seek_y_positions: np.ndarray, y_hat: float, side: Side) -> np.ndarray:
        risk = np.zeros(len(seek_y_positions))
        for idx, seek_y_position in enumerate(seek_y_positions):
            sigma = seek_y_position - y_hat
            if side == Side.RIGHT:
                risk[idx] = self.road_risk_gain * \
                    (-np.arctan(self.road_risk_left_gradient * (sigma + self.road_risk_margin)) + self.road_risk_offset)
            elif side == Side.LEFT:
                risk[idx] = self.road_risk_gain * \
                    (np.arctan(self.road_risk_right_gradient * (sigma + self.road_risk_margin)) + self.road_risk_offset)
        return risk

    def get_benefit_value(self, seek_positions: np.ndarray, y_hat_l: float, y_hat_r: float) -> np.ndarray:
        num_positions = len(seek_positions)
        num_seek_position = len(seek_positions[0][0])

        y_hat_center = (y_hat_l + y_hat_r) * 0.5
        scale = self.benefit_scale
        covariance = [self.benefit_covariance]

        benefits = np.zeros((num_positions, num_seek_position))

        for idx, seek_position in enumerate(seek_positions):
            seek_y_positions = [seek_y_position for seek_y_position in seek_position[1]]

            pdf_values = multivariate_normal.pdf(seek_y_positions, mean=y_hat_center, cov=covariance)
            benefits[idx] = scale * pdf_values

        return benefits
