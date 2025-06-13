import os
import argparse
import torch

from stable_baselines3 import PPO
from torch.utils.tensorboard import SummaryWriter

from env.predict_model_env import StateModelEnv

class RLSteteModel:
    def __init__(self, model_path, total_timesteps=300_000):
        self.train_dir = os.path.dirname(os.path.realpath(__file__))
        self.model_path = model_path
        self.total_timestamps = total_timesteps
        self.writer = SummaryWriter(log_dir="runs/ppo_state_model")

    def learing(self):
        env = StateModelEnv(model_path=self.model_path)
        model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="runs/ppo_state_model")
        model.learn(total_timesteps=self.total_timestamps)

        scripted_model = torch.jit.script(env.model.cpu())
        scripted_model_path = os.path.join(self.train_dir, 'rl_state_model.pt')
        scripted_model.save(scripted_model_path)
        print(f"✅ Saved scripted state model: {scripted_model_path}")
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('model', nargs='?', default=None, help='state predict model_path')
    parser.add_argument('total_timesteps', type=int, nargs='?', default=None, help='limit step')
    args = parser.parse_args()

    total_timesteps = args.total_timesteps if args.total_timesteps is not None else 300_000
    rl = RLSteteModel(args.model, total_timesteps)
    rl.learing()