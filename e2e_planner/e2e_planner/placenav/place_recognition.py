import numpy as np
import torch

from e2e_planner.placenav.topomap import load_topomap


class PlaceRecognition:
    ACTION_TO_COMMAND = {
        'roadside': 0,
        'straight': 1,
        'left': 2,
        'right': 3,
    }

    def __init__(
        self,
        weight_path,
        topomap_path,
        device,
        delta=5.0,
        window_lower=-2,
        window_upper=10,
    ):
        self.device = device
        self.model = torch.jit.load(str(weight_path), map_location=self.device)
        self.model.eval()

        topomap = load_topomap(topomap_path)
        self.node_ids = topomap['node_ids']
        self.actions = topomap['actions']
        self.descriptors = topomap['feature_matrix']

        self.delta = float(delta)
        self.window_lower = int(window_lower)
        self.window_upper = int(window_upper)
        self.transition = np.ones(self.window_upper - self.window_lower, dtype=np.float32)
        self.lambda1 = 0.0
        self.belief = None

    def _compute_distances(self, query_feature):
        dots = np.dot(self.descriptors, query_feature)
        dots = np.clip(dots, -1.0, 1.0)
        return np.sqrt(2.0 - 2.0 * dots)

    def _initialize_belief(self, query_feature):
        dists = self._compute_distances(query_feature)
        descriptor_quantiles = np.quantile(dists, [0.025, 0.975])
        denom = descriptor_quantiles[1] - descriptor_quantiles[0]
        self.lambda1 = np.log(self.delta) / denom if denom > 1e-6 else 1.0
        self.belief = np.exp(-self.lambda1 * dists)
        self.belief /= self.belief.sum()

    def _observation_likelihood(self, query_feature):
        return np.exp(-self.lambda1 * self._compute_distances(query_feature))

    def _update_belief(self, query_feature):
        if self.window_lower < 0:
            conv_ind_l = abs(self.window_lower)
            conv_ind_h = len(self.belief) + abs(self.window_lower)
            bel_ind_l, bel_ind_h = 0, len(self.belief)
        else:
            conv_ind_l, conv_ind_h = 0, len(self.belief) - self.window_lower
            bel_ind_l, bel_ind_h = self.window_lower, len(self.belief)

        belief_pad = np.pad(self.belief, len(self.transition) - 1, mode='symmetric')
        conv = np.convolve(belief_pad, self.transition, mode='valid')
        self.belief[bel_ind_l:bel_ind_h] = conv[conv_ind_l:conv_ind_h]

        if self.window_lower > 0:
            self.belief[:self.window_lower] = 0.0

        self.belief *= self._observation_likelihood(query_feature)
        belief_sum = self.belief.sum()
        if belief_sum <= 0.0:
            self._initialize_belief(query_feature)
        else:
            self.belief /= belief_sum

    def get_recognition(self, image_tensor):
        image_tensor = image_tensor.to(self.device, dtype=torch.float32)
        with torch.no_grad():
            output = self.model(image_tensor)

        query_feature = output.squeeze(0).cpu().numpy().squeeze()

        if self.belief is None:
            self._initialize_belief(query_feature)
        else:
            self._update_belief(query_feature)

        best_idx = int(np.argmax(self.belief))
        action = self.actions[best_idx]
        if action not in self.ACTION_TO_COMMAND:
            raise ValueError(f'Unsupported action in topomap: {action}')

        return self.ACTION_TO_COMMAND[action], best_idx
