import os
import torch
import numpy as np
import rclpy

# 自作パッケージからのインポート
from my_rl_pkg.envs.car_env import GazeboCarEnv  # 前述のGazebo環境クラス
from my_rl_pkg.agents.sac import SACAgent
from my_rl_pkg.utils.replay_buffer import ReplayBuffer

def main():
    # 1. ROS 2 ノードの初期化
    rclpy.init()
    
    # 2. デバイスの設定 (RTXなどのGPUがあれば自動でCUDAを使用)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # 3. 環境のインスタンス化
    env = AiformulaEnv()
    
    # 空間情報の取得 (形状: [Channels, Height, Width] と Action次元)
    state_shape = env.observation_space.shape
    in_channels = state_shape[0]
    action_dim = env.action_space.shape[0]

    print(f"Observation Shape: {state_shape} | Action Dim: {action_dim}")

    # 4. SACエージェントとリプレイバッファの初期化
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

    # 5. 学習用ハイパーパラメータ
    max_episodes = 500
    batch_size = 256
    start_steps = 2000      # 最初はこのステップ数だけランダムに行動してバッファを溜める
    update_after = 2000     # 学習を開始するまでの総ステップ数
    updates_per_step = 1    # 1ステップあたりの勾配更新回数

    total_steps = 0
    
    print("=== Training Start ===")

    for episode in range(1, max_episodes + 1):
        state, info = env.reset()
        episode_reward = 0.0
        episode_steps = 0
        done = False

        while not done:
            # --- アクションの選択 ---
            if total_steps < start_steps:
                # 序盤はランダムなアクションで探索を促す
                action = env.action_space.sample()
            else:
                # SACエージェントによる方策に基づくアクション選択
                action = agent.select_action(state, evaluate=False)

            # --- 環境の1ステップ実行 ---
            next_state, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated

            # --- リプレイバッファへ保存 ---
            memory.add(state, action, next_state, reward, done)

            state = next_state
            episode_reward += reward
            episode_steps += 1
            total_steps += 1

            # --- ネットワークのパラメータ更新 ---
            if total_steps >= update_after:
                for _ in range(updates_per_step):
                    critic_loss, actor_loss, alpha_loss = agent.update_parameters(memory, batch_size)

            # --- ROS 2のメッセージキューをこまめに処理 ---
            if rclpy.ok():
                rclpy.spin_once(env.node, timeout_sec=0.001)

        print(f"Episode: {episode:3d} | Steps: {episode_steps:4d} | Total Steps: {total_steps:6d} | Reward: {episode_reward:8.2f}")

        # --- 定期的なモデルの保存 ---
        if episode % 50 == 0:
            os.makedirs("models", exist_ok=True)
            torch.save(agent.actor.state_dict(), f"models/sac_actor_ep{episode}.pth")
            torch.save(agent.critic.state_dict(), f"models/sac_critic_ep{episode}.pth")
            print(f"--> Checkpoint saved at episode {episode}")

    # 6. クリーンアップ
    env.close()
    print("=== Training Finished ===")

if __name__ == "__main__":
    main()