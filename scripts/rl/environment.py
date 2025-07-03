#!/usr/bin/env python3

import gym
import numpy as np
import torch
import torch.nn as nn
from typing import Tuple, Dict, Any
import math

class ChassisEnvironment(gym.Env):
    """
    状態推定モデルを使用した仮想シャーシ制御環境
    
    State: [v, w, th] (twist単位)
    Action: [left_rpm, right_rpm] (RPM単位, -700~700)
    Observation: [target_linear, target_angular, current_v, current_w, current_th]
    """
    
    def __init__(self, state_model_path: str, episode_length: int = 500, device: str = "auto"):
        super().__init__()
        
        # パラメータ
        self.episode_length = episode_length
        self.max_rpm = 700.0
        self.tread = 0.6  # トレッド幅
        self.wheel_radius = 0.124  # 車輪半径
        self.reduction_ratio = 52.14  # 減速比
        
        # 行動空間: [left_rpm, right_rpm]
        self.action_space = gym.spaces.Box(
            low=-self.max_rpm, 
            high=self.max_rpm, 
            shape=(2,), 
            dtype=np.float32
        )
        
        # 観測空間: [target_linear, target_angular, current_v, current_w, current_th]
        self.observation_space = gym.spaces.Box(
            low=np.array([-2.0, -2.0, -2.0, -2.0, -50.0]),
            high=np.array([2.0, 2.0, 2.0, 2.0, 50.0]),
            dtype=np.float32
        )
        
        # デバイス選択
        if device == "auto":
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        elif device == "cuda":
            if not torch.cuda.is_available():
                print("Warning: CUDA requested but not available. Using CPU.")
                self.device = torch.device("cpu")
            else:
                self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")
        
        # 状態推定モデルの読み込み（堅牢なデバイス処理）
        print(f"Loading state model on {self.device}...")
        try:
            if self.device.type == 'cuda':
                # CUDAの場合、特別な処理でTorchScriptモデルを完全にCUDAに移動
                self.state_model = torch.jit.load(state_model_path, map_location='cpu')
                
                # モデルの全パラメータとバッファをCUDAに移動
                self.state_model = self.state_model.to(self.device)
                
                # LSTMの隠れ状態もCUDAに強制移動
                for name, param in self.state_model.named_parameters():
                    if param.device != self.device:
                        param.data = param.data.to(self.device)
                
                # TorchScriptモデルを再コンパイル（CUDAで）
                dummy_input = torch.randn(1, 10, 5).to(self.device)
                self.state_model = torch.jit.trace(self.state_model, dummy_input)
                
                print(f"✅ Model successfully loaded and recompiled for {self.device}")
            else:
                # CPUの場合は通常の読み込み
                self.state_model = torch.jit.load(state_model_path, map_location=self.device)
                print(f"✅ Model loaded on {self.device}")
            
            self.state_model.eval()
            
        except Exception as e:
            print(f"Warning: Failed to load model on {self.device}: {e}")
            print("Falling back to CPU...")
            self.device = torch.device("cpu")
            self.state_model = torch.jit.load(state_model_path, map_location=self.device)
            self.state_model.eval()
        
        # 環境状態
        self.current_state = np.zeros(3)  # [v, w, th]
        self.target_velocity = np.zeros(2)  # [linear, angular]
        self.step_count = 0
        self.state_history = []  # LSTM用履歴
        self.history_length = 10
        
        print(f"Environment initialized - Device: {self.device}")
    
    def reset(self) -> np.ndarray:
        """環境をリセット"""
        # 初期状態を0に設定
        self.current_state = np.zeros(3)  # [v=0, w=0, th=0]
        
        # 初期速度指令を設定（現実的な値、幅あり）
        self.target_velocity[0] = np.random.uniform(3.0, 5.0)   # linear: 3.0-5.0 m/s
        self.target_velocity[1] = np.random.uniform(-0.1, 0.1)  # angular: ほぼ直進
        
        # 履歴をクリア
        self.state_history = []
        self.step_count = 0
        
        return self._get_observation()
    
    def _set_realistic_target_velocity(self):
        """現実的な速度指令を設定（範囲指定）"""
        patterns = [
            ([3.0, 5.0], [-0.1, 0.1]),     # 直進（幅あり）
            ([1.5, 2.5], [0.3, 0.7]),      # 左旋回
            ([1.5, 2.5], [-0.7, -0.3]),    # 右旋回
            ([4.5, 6.0], [-0.05, 0.05]),   # 高速直進
            ([-0.1, 0.1], [-0.1, 0.1]),    # 停止付近
            ([1.0, 2.0], [0.4, 0.8]),      # 低速左旋回
            ([1.0, 2.0], [-0.8, -0.4]),    # 低速右旋回
        ]
        
        # ランダムにパターンを選択し、範囲内でランダムに値を決定
        linear_range, angular_range = patterns[np.random.randint(len(patterns))]
        
        self.target_velocity[0] = np.random.uniform(linear_range[0], linear_range[1])
        self.target_velocity[1] = np.random.uniform(angular_range[0], angular_range[1])
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        """一ステップ実行"""
        # アクションのクリッピング
        left_rpm = np.clip(action[0], -self.max_rpm, self.max_rpm)
        right_rpm = np.clip(action[1], -self.max_rpm, self.max_rpm)
        
        # 状態推定モデルで次の状態を予測
        next_state = self._predict_next_state(left_rpm, right_rpm)
        
        # 報酬計算
        reward = self._calculate_reward(next_state, left_rpm, right_rpm)
        
        # 状態更新
        self.current_state = next_state
        self.step_count += 1
        
        # 終了判定
        done = self.step_count >= self.episode_length or self._is_terminal()
        
        # 新しい目標速度を設定（一定間隔で変更）
        if self.step_count % 200 == 0:
            self._set_realistic_target_velocity()
        
        info = {
            'velocity_error': np.linalg.norm(self.target_velocity - self.current_state[:2]),
            'rpm_left': left_rpm,
            'rpm_right': right_rpm
        }
        
        return self._get_observation(), reward, done, info
    
    def _predict_next_state(self, left_rpm: float, right_rpm: float) -> np.ndarray:
        """状態推定モデルを使用して次の状態を予測"""
        # 現在の状態と指令を履歴に追加
        current_input = [
            self.current_state[0],  # v
            self.current_state[1],  # w
            self.current_state[2],  # th
            left_rpm,               # rpm_l
            right_rpm               # rpm_r
        ]
        
        self.state_history.append(current_input)
        if len(self.state_history) > self.history_length:
            self.state_history.pop(0)
        
        # LSTM用のシーケンス準備
        seq_len = len(self.state_history)
        if seq_len == 0:
            return self.current_state.copy()
        
        # 足りない分は最初の値で埋める
        padded_history = []
        if seq_len < self.history_length:
            for _ in range(self.history_length - seq_len):
                padded_history.append(self.state_history[0])
        padded_history.extend(self.state_history)
        
        # Tensorに変換
        input_tensor = torch.tensor(
            padded_history, dtype=torch.float32, device=self.device
        ).unsqueeze(0)  # バッチ次元追加
        
        # 推論実行
        with torch.no_grad():
            output = self.state_model(input_tensor)
            predicted_state = output.cpu().numpy()[0]  # [v1, w1, th1]
        
        return predicted_state
    
    def _calculate_reward(self, next_state: np.ndarray, left_rpm: float, right_rpm: float) -> float:
        """報酬関数"""
        # 速度誤差ペナルティ
        velocity_error = np.linalg.norm(self.target_velocity - next_state[:2])
        velocity_reward = -velocity_error * 10.0
        
        # RPM効率性ペナルティ（過度なRPMを避ける）
        rpm_penalty = -(abs(left_rpm) + abs(right_rpm)) * 0.001
        
        # 安定性ボーナス（急激な変化を避ける）
        state_change = np.linalg.norm(next_state - self.current_state)
        stability_bonus = -state_change * 1.0 if state_change > 0.5 else 0.0
        
        # 目標追従ボーナス
        if velocity_error < 0.1:
            tracking_bonus = 5.0
        elif velocity_error < 0.2:
            tracking_bonus = 2.0
        else:
            tracking_bonus = 0.0
        
        total_reward = velocity_reward + rpm_penalty + stability_bonus + tracking_bonus
        
        return total_reward
    
    def _is_terminal(self) -> bool:
        """終了条件の判定"""
        # 異常値検出
        if np.any(np.abs(self.current_state[:2]) > 5.0):  # 速度が異常
            return True
        if abs(self.current_state[2]) > 100.0:  # ポテンショが異常
            return True
        
        return False
    
    def _get_observation(self) -> np.ndarray:
        """観測値を取得"""
        obs = np.concatenate([
            self.target_velocity,  # [target_linear, target_angular]
            self.current_state     # [current_v, current_w, current_th]
        ])
        return obs.astype(np.float32)
    
    def render(self, mode='human'):
        """レンダリング（実装省略）"""
        if mode == 'human':
            print(f"Step: {self.step_count}, State: {self.current_state}, Target: {self.target_velocity}")