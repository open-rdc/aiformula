import numpy as np
import torch
import time
from gym import Env, spaces

class StateModelEnv(Env):
    def __init__(self, model_path, seq_len=10):
        super().__init__()

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # LSTMモデル読み込み
        self.model = torch.jit.load(model_path).to(self.device)
        self.model.eval()

        self.seq_len = seq_len
        self.input_dim = 7  # [v, w, acc_v, acc_w, potentiometer, linear_x, angular_z]
        self.output_dim = 3  # [v, w, potentiometer]

        # Gym仕様
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.seq_len, self.input_dim), dtype=np.float32)
        self.action_space = spaces.Box(low=np.array([-1.0, -2.0]), high=np.array([1.0, 2.0]), dtype=np.float32)

        self.last_time = None
        self.state_buffer = None  # shape: [seq_len, input_dim]
        self.cmd_vel_target = np.zeros(2, dtype=np.float32)  # [linear_x, angular_z]

    def reset(self):
        self.state_buffer = np.zeros((self.seq_len, self.input_dim), dtype=np.float32)
        self.last_time = time.perf_counter()

        # 新しい目標cmd_velをランダムに設定
        self.cmd_vel_target = np.array([
            np.random.uniform(0.0, 5.0),    # linear_x
            np.random.uniform(-1.0, 1.0)    # angular_z
        ], dtype=np.float32)

        return self.state_buffer

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        current_time = time.perf_counter()
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt == 0.0: dt = 1e-6

        # 現在の状態から [v, w, potentio] を取得
        prev = self.state_buffer[-1]
        v_prev, w_prev, acc_v_prev, acc_w_prev, pth, *_ = prev

        # LSTM 推論
        new_input = np.array([
            v_prev, w_prev, acc_v_prev, acc_w_prev, pth, action[0], action[1]
        ], dtype=np.float32)
        input_seq = np.vstack([self.state_buffer[1:], new_input])
        input_tensor = torch.tensor(input_seq, dtype=torch.float32).unsqueeze(0).to(self.device)

        with torch.no_grad():
            predicted = self.model(input_tensor).cpu().numpy().squeeze()

        v_next, w_next, pth_next = predicted[0], predicted[1], predicted[2]
        acc_v = (v_next - v_prev)/dt
        acc_w = (w_next - w_prev)/dt

        reward = -np.linalg.norm(np.array([v_next, w_next]) - self.cmd_vel_target)

        # 状態更新
        updated_input = new_input.copy()
        updated_input[0] = v_next
        updated_input[1] = w_next
        updated_input[2] = acc_v
        updated_input[3] = acc_w
        updated_input[4] = pth_next

        self.state_buffer = np.vstack([self.state_buffer[1:], updated_input])

        done = False
        info = {"v_pred": v_next, "w_pred": w_next, "target": self.cmd_vel_target}

        return self.state_buffer, reward, done, info
