#!/usr/bin/env python3

import os
import sys
import argparse
import numpy as np
import torch
import matplotlib.pyplot as plt
from datetime import datetime
import json
from typing import Dict, List
from tqdm import tqdm
from torch.utils.tensorboard import SummaryWriter

# モジュールパスを追加
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from environment import ChassisEnvironment
from ppo_agent import PPOAgent

class TrainingManager:
    """強化学習の訓練管理クラス"""
    
    def __init__(self, config: Dict):
        self.config = config
        
        # 環境とエージェントの初期化
        self.env = ChassisEnvironment(
            state_model_path=config['state_model_path'],
            episode_length=config['episode_length'],
            device=config.get('device', 'auto')
        )
        
        self.agent = PPOAgent(
            state_dim=config['state_dim'],
            action_dim=config['action_dim'],
            lr=config['learning_rate'],
            gamma=config['gamma'],
            gae_lambda=config['gae_lambda'],
            clip_epsilon=config['clip_epsilon'],
            value_coef=config['value_coef'],
            entropy_coef=config['entropy_coef'],
            device=config.get('device', 'auto')
        )
        
        # ログ管理
        self.episode_rewards = []
        self.episode_lengths = []
        self.velocity_errors = []
        self.loss_history = []
        
        # 保存ディレクトリ
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_dir = f"rl_results_{timestamp}"
        os.makedirs(self.save_dir, exist_ok=True)
        
        # TensorBoard設定
        self.tensorboard_dir = os.path.join(self.save_dir, 'tensorboard')
        self.writer = SummaryWriter(log_dir=self.tensorboard_dir)
        
        # 設定を保存
        with open(os.path.join(self.save_dir, 'config.json'), 'w') as f:
            json.dump(config, f, indent=2)
        
        print(f"Training manager initialized - Save dir: {self.save_dir}")
        print(f"TensorBoard logs: {self.tensorboard_dir}")
        print(f"View with: tensorboard --logdir {self.tensorboard_dir}")
    
    def train(self):
        """メインの訓練ループ"""
        print("Starting RL training...")
        
        best_reward = float('-inf')
        
        for episode in tqdm(range(self.config['num_episodes']), desc="Training Episodes"):
            episode_reward, episode_length, avg_velocity_error = self._run_episode(episode)
            
            # ログ記録
            self.episode_rewards.append(episode_reward)
            self.episode_lengths.append(episode_length)
            self.velocity_errors.append(avg_velocity_error)
            
            # 50エピソード毎にTensorBoardログ（平均値）
            if episode % 50 == 0 and episode > 0:
                recent_rewards = self.episode_rewards[-50:]
                recent_lengths = self.episode_lengths[-50:]
                recent_errors = self.velocity_errors[-50:]
                
                self.writer.add_scalar('Episode/Reward_Avg50', np.mean(recent_rewards), episode)
                self.writer.add_scalar('Episode/Length_Avg50', np.mean(recent_lengths), episode)
                self.writer.add_scalar('Episode/VelocityError_Avg50', np.mean(recent_errors), episode)
            
            # 定期的にモデル保存
            if episode % self.config['save_interval'] == 0:
                model_path = os.path.join(self.save_dir, f'model_episode_{episode}.pt')
                self.agent.save(model_path)
                
                # 最良モデルの保存
                if episode_reward > best_reward:
                    best_reward = episode_reward
                    best_model_path = os.path.join(self.save_dir, 'best_model.pt')
                    self.agent.save(best_model_path)
            
            # 進捗表示
            if episode % self.config['log_interval'] == 0:
                recent_rewards = self.episode_rewards[-self.config['log_interval']:]
                recent_errors = self.velocity_errors[-self.config['log_interval']:]
                
                print(f"Episode {episode}: "
                      f"Avg Reward: {np.mean(recent_rewards):.2f}, "
                      f"Avg Velocity Error: {np.mean(recent_errors):.4f}, "
                      f"Best Reward: {best_reward:.2f}")
        
        # 最終モデル保存
        final_model_path = os.path.join(self.save_dir, 'final_model.pt')
        self.agent.save(final_model_path)
        
        # 結果の可視化と保存
        self._save_results()
        
        # TensorBoard writerを閉じる
        self.writer.close()
        
        print(f"Training completed! Results saved to {self.save_dir}")
        print(f"View TensorBoard with: tensorboard --logdir {self.tensorboard_dir}")
    
    def _run_episode(self, episode_num: int) -> tuple:
        """単一エピソードの実行"""
        state = self.env.reset()
        episode_reward = 0
        velocity_errors = []
        step_count = 0
        
        while True:
            # 行動選択
            action, log_prob, value = self.agent.select_action(state)
            
            # 環境でステップ実行
            next_state, reward, done, info = self.env.step(action)
            
            # 経験保存
            self.agent.store_transition(state, action, reward, log_prob, value, done)
            
            # 統計更新
            episode_reward += reward
            velocity_errors.append(info['velocity_error'])
            step_count += 1
            
            state = next_state
            
            if done:
                break
        
        # PPO更新
        if len(self.agent.states) >= self.config['batch_size']:
            # 最終状態での価値推定
            final_state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.agent.device)
            with torch.no_grad():
                _, _, final_value = self.agent.actor_critic.forward(final_state_tensor)
                final_value = final_value.cpu().numpy()[0]
            
            loss_info = self.agent.update(next_value=final_value if not done else 0.0)
            if loss_info:
                self.loss_history.append(loss_info)
                
                # 50エピソード毎にTensorBoardに損失情報をログ
                if episode_num % 50 == 0 and episode_num > 0:
                    recent_losses = self.loss_history[-10:] if len(self.loss_history) >= 10 else self.loss_history
                    if recent_losses:
                        avg_total_loss = np.mean([loss['total_loss'] for loss in recent_losses])
                        avg_policy_loss = np.mean([loss['policy_loss'] for loss in recent_losses])
                        avg_value_loss = np.mean([loss['value_loss'] for loss in recent_losses])
                        avg_entropy_loss = np.mean([loss['entropy_loss'] for loss in recent_losses])
                        
                        self.writer.add_scalar('Loss/Total_Avg', avg_total_loss, episode_num)
                        self.writer.add_scalar('Loss/Policy_Avg', avg_policy_loss, episode_num)
                        self.writer.add_scalar('Loss/Value_Avg', avg_value_loss, episode_num)
                        self.writer.add_scalar('Loss/Entropy_Avg', avg_entropy_loss, episode_num)
        
        avg_velocity_error = np.mean(velocity_errors) if velocity_errors else 0.0
        
        return episode_reward, step_count, avg_velocity_error
    
    def _save_results(self):
        """結果の保存と可視化"""
        # 数値データ保存
        results = {
            'episode_rewards': self.episode_rewards,
            'episode_lengths': self.episode_lengths,
            'velocity_errors': self.velocity_errors,
            'loss_history': self.loss_history
        }
        
        results_path = os.path.join(self.save_dir, 'training_results.json')
        with open(results_path, 'w') as f:
            json.dump(results, f, indent=2)
        
        # グラフ作成
        self._plot_training_curves()
    
    def _plot_training_curves(self):
        """訓練曲線のプロット"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # エピソード報酬
        axes[0, 0].plot(self.episode_rewards)
        axes[0, 0].set_title('Episode Rewards')
        axes[0, 0].set_xlabel('Episode')
        axes[0, 0].set_ylabel('Reward')
        axes[0, 0].grid(True)
        
        # 速度誤差
        axes[0, 1].plot(self.velocity_errors)
        axes[0, 1].set_title('Velocity Tracking Error')
        axes[0, 1].set_xlabel('Episode')
        axes[0, 1].set_ylabel('Average Velocity Error')
        axes[0, 1].grid(True)
        
        # エピソード長
        axes[1, 0].plot(self.episode_lengths)
        axes[1, 0].set_title('Episode Lengths')
        axes[1, 0].set_xlabel('Episode')
        axes[1, 0].set_ylabel('Steps')
        axes[1, 0].grid(True)
        
        # 損失履歴
        if self.loss_history:
            policy_losses = [loss['policy_loss'] for loss in self.loss_history]
            value_losses = [loss['value_loss'] for loss in self.loss_history]
            
            axes[1, 1].plot(policy_losses, label='Policy Loss')
            axes[1, 1].plot(value_losses, label='Value Loss')
            axes[1, 1].set_title('Training Losses')
            axes[1, 1].set_xlabel('Update')
            axes[1, 1].set_ylabel('Loss')
            axes[1, 1].legend()
            axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.save_dir, 'training_curves.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def evaluate(self, num_episodes: int = 10):
        """訓練されたモデルの評価"""
        print(f"Evaluating model for {num_episodes} episodes...")
        
        eval_rewards = []
        eval_errors = []
        
        for episode in range(num_episodes):
            state = self.env.reset()
            episode_reward = 0
            velocity_errors = []
            
            while True:
                # 決定的行動選択
                action, _, _ = self.agent.select_action(state, deterministic=True)
                next_state, reward, done, info = self.env.step(action)
                
                episode_reward += reward
                velocity_errors.append(info['velocity_error'])
                
                state = next_state
                
                if done:
                    break
            
            eval_rewards.append(episode_reward)
            eval_errors.append(np.mean(velocity_errors))
        
        print(f"Evaluation Results:")
        print(f"  Average Reward: {np.mean(eval_rewards):.2f} ± {np.std(eval_rewards):.2f}")
        print(f"  Average Velocity Error: {np.mean(eval_errors):.4f} ± {np.std(eval_errors):.4f}")
        
        return eval_rewards, eval_errors

def main():
    parser = argparse.ArgumentParser(description='Chassis RL Training')
    parser.add_argument('--state-model', type=str, required=True, 
                       help='Path to the state estimation model (.pt file)')
    parser.add_argument('--episodes', type=int, default=1000, 
                       help='Number of training episodes')
    parser.add_argument('--lr', type=float, default=3e-4, 
                       help='Learning rate')
    parser.add_argument('--evaluate', action='store_true', 
                       help='Run evaluation after training')
    parser.add_argument('--device', type=str, default='auto', choices=['auto', 'cpu', 'cuda'],
                       help='Device to use (auto, cpu, cuda)')
    
    args = parser.parse_args()
    
    # 設定
    config = {
        'state_model_path': args.state_model,
        'num_episodes': args.episodes,
        'episode_length': 50,
        'state_dim': 5,
        'action_dim': 2,
        'learning_rate': args.lr,
        'gamma': 0.99,
        'gae_lambda': 0.95,
        'clip_epsilon': 0.2,
        'value_coef': 0.5,
        'entropy_coef': 0.01,
        'batch_size': 64,
        'save_interval': 100,
        'log_interval': 50,
        'device': args.device
    }
    
    # 訓練実行
    trainer = TrainingManager(config)
    trainer.train()
    
    # 評価実行
    if args.evaluate:
        trainer.evaluate()

if __name__ == '__main__':
    main()