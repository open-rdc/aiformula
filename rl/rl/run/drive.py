import argparse
import math
import os
import sys

import rclpy
import torch

from rl.envs.aiformula_env import AiformulaEnv, EnvConfig
from rl.net.network import Actor


def load_actor(path: str, in_channels: int, action_dim: int, device: torch.device) -> Actor:
    data = torch.load(os.path.expanduser(path), map_location=device)
    if 'actor' in data:
        data = data['actor']
    actor = Actor(in_channels, action_dim).to(device)
    actor.load_state_dict(data)
    actor.eval()
    return actor


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, help="sac_actor_*.pth か sac_ckpt_*.pth")
    parser.add_argument("--real", action="store_true", help="実機で走らせる（省略時はシム）")
    parser.add_argument("--no-stop-on-departure", action="store_true",
                        help="脱線を検出しても止まらずに走り続ける（省略時は止まって Share を待つ）")
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init(args=sys.argv, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    env = AiformulaEnv(EnvConfig(sim=not args.real, max_episode_sec=math.inf))
    actor = load_actor(args.model, env.observation_space.shape[0], env.action_space.shape[0], device)
    print(f"Loaded {args.model} on {device}")

    try:
        state, _ = env.reset()
        while True:
            with torch.no_grad():
                x = torch.as_tensor(state, dtype=torch.float32, device=device).unsqueeze(0) / 255.0
                _, _, action = actor.sample(x)
            state, _, terminated, truncated, info = env.step(action.cpu().numpy()[0])

            if info["intervened"]:
                print("Share で手動に切り替わりました。Share で自動にすると再開します")
                state, _ = env.reset()
            elif terminated:
                if args.no_stop_on_departure:
                    env.departure = 0
                    print(f"脱線を検出しました（{info['departure']:+d}）。走行を続けます")
                else:
                    print(f"脱線を検出したので止まりました（{info['departure']:+d}）")
                    state, _ = env.reset()
    except KeyboardInterrupt:
        print("\nCtrl-C: 停止します")
    finally:
        env.close()


if __name__ == "__main__":
    main()
