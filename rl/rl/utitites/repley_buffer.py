import numpy as np
import torch

class ReplayBuffer:
    def __init__(self, capacity: int, state_shape: tuple, action_dim: int):
        self.capacity = capacity
        self.ptr = 0
        self.size = 0

        # 画像はメモリ節約のため uint8 で保持
        self.states = np.zeros((capacity, *state_shape), dtype=np.uint8)
        self.next_states = np.zeros((capacity, *state_shape), dtype=np.uint8)
        self.actions = np.zeros((capacity, action_dim), dtype=np.float32)
        self.rewards = np.zeros((capacity, 1), dtype=np.float32)
        self.not_dones = np.zeros((capacity, 1), dtype=np.float32) # truncated考慮用

    def add(self, state, action, next_state, reward, done):
        self.states[self.ptr] = state
        self.next_states[self.ptr] = next_state
        self.actions[self.ptr] = action
        self.rewards[self.ptr] = reward
        self.not_dones[self.ptr] = 1.0 - float(done)

        self.ptr = (self.ptr + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size: int, device: torch.device):
        ind = np.random.randint(0, self.size, size=batch_size)

        # サンプリング時に float32 に変換しつつ [0.0, 1.0] にスケーリング
        state = torch.FloatTensor(self.states[ind]).to(device) / 255.0
        next_state = torch.FloatTensor(self.next_states[ind]).to(device) / 255.0
        action = torch.FloatTensor(self.actions[ind]).to(device)
        reward = torch.FloatTensor(self.rewards[ind]).to(device)
        not_done = torch.FloatTensor(self.not_dones[ind]).to(device)

        return state, action, next_state, reward, not_done