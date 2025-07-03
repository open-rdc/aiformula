#!/usr/bin/env python3

import torch
import torch.nn as nn
import argparse
import sys
import os

class ChassisControlModel(nn.Module):
    """
    chassis_driverで使用するための制御モデル
    入力: [cmd_linear, cmd_angular, obs_v, obs_w, obs_th] (5次元)
    出力: [left_rpm, right_rpm] (2次元)
    """
    
    def __init__(self, hidden_dim: int = 256):
        super().__init__()
        
        self.network = nn.Sequential(
            nn.Linear(5, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(),
            nn.Linear(hidden_dim // 2, 2),
            nn.Tanh()  # [-1, 1]に正規化
        )
    
    def forward(self, x):
        output = self.network(x)
        # [-700, 700] RPMにスケール
        return output * 700.0

def convert_ppo_to_chassis_model(ppo_model_path: str, output_path: str):
    """
    PPOモデルから chassis_driver用のモデルに変換
    """
    print(f"Loading PPO model from: {ppo_model_path}")
    
    # PPOモデルの読み込み
    device = torch.device("cpu")  # chassis_driverとの互換性のためCPU
    checkpoint = torch.load(ppo_model_path, map_location=device)
    
    # PPOのActorCriticから Actorの重みを抽出
    actor_critic_state = checkpoint['actor_critic_state_dict']
    
    # 新しいモデルを作成
    chassis_model = ChassisControlModel()
    
    # 重みをマッピング
    chassis_state_dict = {}
    
    # 特徴抽出層
    chassis_state_dict['network.0.weight'] = actor_critic_state['feature_extractor.0.weight']
    chassis_state_dict['network.0.bias'] = actor_critic_state['feature_extractor.0.bias']
    chassis_state_dict['network.2.weight'] = actor_critic_state['feature_extractor.2.weight']
    chassis_state_dict['network.2.bias'] = actor_critic_state['feature_extractor.2.bias']
    
    # Actor部分
    chassis_state_dict['network.4.weight'] = actor_critic_state['actor_mean.0.weight']
    chassis_state_dict['network.4.bias'] = actor_critic_state['actor_mean.0.bias']
    chassis_state_dict['network.6.weight'] = actor_critic_state['actor_mean.2.weight']
    chassis_state_dict['network.6.bias'] = actor_critic_state['actor_mean.2.bias']
    
    # モデルに重みを読み込み
    chassis_model.load_state_dict(chassis_state_dict)
    chassis_model.eval()
    
    print("Model conversion successful")
    
    # TorchScriptとして保存（chassis_driverで使用するため）
    example_input = torch.randn(1, 5)
    traced_model = torch.jit.trace(chassis_model, example_input)
    traced_model.save(output_path)
    
    print(f"Chassis control model saved to: {output_path}")
    
    # テスト実行
    print("Testing converted model...")
    test_input = torch.tensor([[0.5, 0.2, 0.3, 0.1, 10.0]])  # [cmd_linear, cmd_angular, obs_v, obs_w, obs_th]
    
    with torch.no_grad():
        output = traced_model(test_input)
        print(f"Test input: {test_input.numpy()[0]}")
        print(f"Test output (RPM): {output.numpy()[0]}")

def main():
    parser = argparse.ArgumentParser(description='Convert PPO model to chassis control model')
    parser.add_argument('ppo_model', type=str, help='Path to PPO model (.pt file)')
    parser.add_argument('--output', type=str, default='chassis_control_model.pt',
                       help='Output path for chassis model')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.ppo_model):
        print(f"Error: PPO model file not found: {args.ppo_model}")
        sys.exit(1)
    
    convert_ppo_to_chassis_model(args.ppo_model, args.output)

if __name__ == '__main__':
    main()