import argparse
import os
import sys
import time
import torch
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory

from rl.envs.aiformula_env import AiformulaEnv, EnvConfig
from rl.agents.sac import SACAgent
from rl.utitites.repley_buffer import ReplayBuffer

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--real", action="store_true", help="実機で学習する（省略時はシム）")
    parser.add_argument("--topomap", action="store_true", help="報酬にトポロジカルマップのノード到達を使う（省略時は物体検出）")
    parser.add_argument("--model-dir", default="models", help="モデルの保存先（launch からは絶対パスを渡す）")
    parser.add_argument("--auto-reset", action="store_true", help="シムで車を自動で初期位置に戻す（省略時は Share で手動にして人が戻し、Share で自動に戻したら再開）")
    parser.add_argument("--episode-sec", type=float, default=300.0, help="エピソードの制限時間 [s]（既定 5 分）")
    parser.add_argument("--resume", default=None,
                        help="再開するモデル。sac_ckpt_*.pth（Ctrl-C で保存した全部入り）か sac_actor_*.pth（重みだけ）")
    parser.add_argument("--save-name", default="", help="保存ファイル名。同じ名前のファイルがあれば上書きする")
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])

    rclpy.init(args=sys.argv, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    topomap_path = os.path.join(get_package_share_directory("rl"), "config", "aiformula_topomap.osm") if args.topomap else None
    env = AiformulaEnv(EnvConfig(sim=not args.real, topomap_path=topomap_path, auto_reset=args.auto_reset,
                                max_episode_sec=args.episode_sec))
    
    state_shape = env.observation_space.shape
    in_channels = state_shape[0]
    action_dim = env.action_space.shape[0]

    print(f"Observation Shape: {state_shape} | Action Dim: {action_dim}")

    agent = SACAgent(
        in_channels=in_channels,
        action_dim=action_dim,
        device=device,
        lr=3e-4,
        gamma=0.99,
        tau=0.005
    )
    
    buffer_capacity = 100_000
    memory = ReplayBuffer(
        capacity=buffer_capacity,
        state_shape=state_shape,
        action_dim=action_dim
    )

    max_episodes = 500
    batch_size = 256
    start_steps = 2000
    update_after = 2000
    updates_per_step = 1

    total_steps = 0

    start_episode = 1
    if args.resume:
        path = os.path.expanduser(args.resume)
        data = torch.load(path, map_location=device)
        if 'actor' in data:
            agent.load_checkpoint(data)
            start_episode = data.get('episode', 0) + 1
            total_steps = data.get('total_steps', 0)
            print(f"Resumed from checkpoint {path} (episode {start_episode - 1}, total steps {total_steps})")
        else:
            critic_path = os.path.join(os.path.dirname(path), os.path.basename(path).replace('sac_actor_', 'sac_critic_', 1))
            critic = torch.load(critic_path, map_location=device) if critic_path != path and os.path.exists(critic_path) else None
            agent.load_networks(data, critic)
            print(f"Resumed weights from {path}" + (f" + {critic_path}" if critic is not None else " (critic は初期値から)"))
        start_steps = 0
    run_steps = 0

    print("=== Training Start ===")

    episode = start_episode - 1
    try:
        for episode in range(start_episode, start_episode + max_episodes):
            state, info = env.reset()
            episode_reward = 0.0
            episode_steps = 0
            done = False

            while not done:
                if run_steps < start_steps:
                    action = env.action_space.sample()
                else:
                    action = agent.select_action(state, evaluate=False)

                next_state, reward, terminated, truncated, info = env.step(action)
                done = terminated or truncated

                memory.add(state, action, next_state, reward, terminated)

                state = next_state
                episode_reward += reward
                episode_steps += 1
                total_steps += 1
                run_steps += 1

                if run_steps >= update_after:
                    for _ in range(updates_per_step):
                        critic_loss, actor_loss, alpha_loss = agent.update_parameters(memory, batch_size)


            print(f"Episode: {episode:3d} | Steps: {episode_steps:4d} | Total Steps: {total_steps:6d} | Reward: {episode_reward:8.2f}")

    except KeyboardInterrupt:
        os.makedirs(args.model_dir, exist_ok=True)
        name = args.save_name or f"{time.strftime('%Y%m%d_%H%M%S')}_ep{episode}"
        actor_path = os.path.join(args.model_dir, f"sac_actor_{name}.pth")
        ckpt_path = os.path.join(args.model_dir, f"sac_ckpt_{name}.pth")
        if os.path.exists(ckpt_path):
            print(f"\n--> {name} は既にあるので上書きします")
        torch.save(agent.actor.state_dict(), actor_path)
        torch.save({**agent.checkpoint(), 'episode': episode, 'total_steps': total_steps}, ckpt_path)
        print(f"\n--> Ctrl-C: saved {actor_path}")
        print(f"--> Ctrl-C: saved {ckpt_path} (episode {episode}, total steps {total_steps})")
        print(f"    再開: ros2 launch rl train.launch.py resume:={ckpt_path}")

    env.close()
    print("=== Training Finished ===")

if __name__ == "__main__":
    main()