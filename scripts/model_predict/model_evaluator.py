#!/usr/bin/env python3

import torch
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # GUI表示用のバックエンドを設定
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score
import argparse
import os
from pathlib import Path
import struct
from collections import deque

# ROSbag読み込み関係
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from socketcan_interface_msg.msg import SocketcanIF

def bytes_to_short(byte_list):
    return struct.unpack('<h', bytearray(byte_list[:2]))[0]

class ModelEvaluator:
    def __init__(self, model_path):
        """学習済みモデルの読み込み"""
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found: {model_path}")
        
        # CPUで実行するように設定
        self.device = torch.device('cpu')
        self.model = torch.jit.load(model_path, map_location=self.device)
        self.model.eval()
        print(f"学習済みモデルを読み込みました: {model_path}")
        print(f"デバイス: {self.device}")
        
    def load_rosbag_data(self, bag_path, seq_len=10):
        """ROSbagからデータを読み込み、時系列データを作成"""
        reader = SequentialReader()
        storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        reader.open(storage_options, converter_options)

        valid_topics = {
            '/vectornav/imu': Imu,
            '/vectornav/velocity_body': TwistWithCovarianceStamped,
            '/can_rx_011': SocketcanIF,
            '/cmd_vel': Twist
        }

        type_map = {}
        for topic in reader.get_all_topics_and_types():
            if topic.name in valid_topics:
                type_map[topic.name] = get_message(topic.type)

        latest_vel = None
        latest_acc = None
        latest_potentio = None
        latest_cmd = None

        # データ保存用
        history = deque(maxlen=2)
        all_samples = []

        print("ROSbagからデータを読み込み中...")
        
        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic not in type_map:
                continue
            msg_type = type_map[topic]
            msg = deserialize_message(data, msg_type)

            if topic == '/vectornav/imu':
                latest_acc = {
                    'acc_v': msg.linear_acceleration.x,
                    'acc_w': msg.angular_velocity.z
                }

            elif topic == '/vectornav/velocity_body':
                latest_vel = {
                    'v': msg.twist.twist.linear.x,
                    'w': msg.twist.twist.angular.z
                }

            elif topic == '/can_rx_011':
                latest_potentio = {
                    'potentio_th': bytes_to_short(msg.candata[:2])
                }

            elif topic == '/cmd_vel':
                latest_cmd = {
                    'linear_x': msg.linear.x,
                    'angular_z': msg.angular.z
                }

                # すべてのセンサが揃っているときのみ記録
                if latest_vel and latest_acc and latest_potentio and latest_cmd:
                    record = {
                        'v': latest_vel['v'],
                        'w': latest_vel['w'],
                        'acc_v': latest_acc['acc_v'],
                        'acc_w': latest_acc['acc_w'],
                        'potentio_th': latest_potentio['potentio_th'],
                        'linear_x': latest_cmd['linear_x'],
                        'angular_z': latest_cmd['angular_z']
                    }
                    history.append(record)

                    if len(history) == 2:
                        x = history[0]
                        y = history[1]
                        
                        # 入力データ (v0, w0, acc_v0, acc_w0, potentio_th0, linear_x, angular_z)
                        input_data = [
                            x['v'], x['w'], x['acc_v'], x['acc_w'], x['potentio_th'],
                            x['linear_x'], x['angular_z']
                        ]
                        
                        # 真値 (v1, w1, potentio_th1)
                        target_data = [y['v'], y['w'], y['potentio_th']]
                        
                        all_samples.append({
                            'input': input_data,
                            'target': target_data
                        })

        print(f"基本データ読み込み完了: {len(all_samples)}件のサンプル")
        
        # 時系列データを作成 (seq_len分のデータをまとめる)
        sequence_data = []
        for i in range(len(all_samples) - seq_len):
            input_seq = [all_samples[j]['input'] for j in range(i, i + seq_len)]
            target = all_samples[i + seq_len]['target']
            sequence_data.append({
                'input': input_seq,
                'target': target
            })

        print(f"時系列データ作成完了: {len(sequence_data)}件のシーケンス")
        return sequence_data

    def predict(self, input_data):
        """モデル推論実行"""
        # 入力データは既に(batch_size, seq_len, features)の形状になっている
        input_tensor = torch.FloatTensor(input_data).to(self.device)
        
        print(f"入力テンソル形状: {input_tensor.shape}")
        
        with torch.no_grad():
            predictions = self.model(input_tensor)
        
        return predictions.cpu().numpy()

    def calculate_metrics(self, y_true, y_pred):
        """評価指標計算"""
        metrics = {}
        columns = ['v', 'w', 'potentio_th']
        
        for i, column in enumerate(columns):
            mae = mean_absolute_error(y_true[:, i], y_pred[:, i])
            rmse = np.sqrt(mean_squared_error(y_true[:, i], y_pred[:, i]))
            r2 = r2_score(y_true[:, i], y_pred[:, i])
            max_error = np.max(np.abs(y_true[:, i] - y_pred[:, i]))
            
            # 相関係数
            correlation = np.corrcoef(y_true[:, i], y_pred[:, i])[0, 1]
            
            metrics[column] = {
                'MAE': mae,
                'RMSE': rmse,
                'R²': r2,
                'Max_Error': max_error,
                'Correlation': correlation
            }
        
        return metrics

    def visualize_results(self, y_true, y_pred, metrics):
        """結果可視化 - plt.show()を使用"""
        columns = ['v (Linear Velocity)', 'w (Angular Velocity)', 'potentio_th (Steering Angle)']
        column_keys = ['v', 'w', 'potentio_th']
        
        # 1. 散布図 (真値 vs 推定値)
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        fig.suptitle('True vs Predicted Values', fontsize=16)
        
        for i, (col, key) in enumerate(zip(columns, column_keys)):
            axes[i].scatter(y_true[:, i], y_pred[:, i], alpha=0.6, s=1)
            
            # 理想ライン (y=x)
            min_val = min(y_true[:, i].min(), y_pred[:, i].min())
            max_val = max(y_true[:, i].max(), y_pred[:, i].max())
            axes[i].plot([min_val, max_val], [min_val, max_val], 'r--', lw=2)
            
            axes[i].set_xlabel(f'True {col}')
            axes[i].set_ylabel(f'Predicted {col}')
            axes[i].set_title(f'{col}\nR² = {metrics[key]["R²"]:.3f}')
            axes[i].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()

        # 2. 時系列プロット
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle('Time Series Comparison', fontsize=16)
        
        time_steps = np.arange(len(y_true))
        
        for i, (col, key) in enumerate(zip(columns, column_keys)):
            axes[i].plot(time_steps, y_true[:, i], label='True', alpha=0.7, linewidth=1)
            axes[i].plot(time_steps, y_pred[:, i], label='Predicted', alpha=0.7, linewidth=1)
            axes[i].set_ylabel(col)
            axes[i].set_title(f'{col} - MAE: {metrics[key]["MAE"]:.4f}')
            axes[i].legend()
            axes[i].grid(True, alpha=0.3)
        
        axes[-1].set_xlabel('Time Steps')
        plt.tight_layout()
        plt.show()

        # 3. 誤差分布ヒストグラム
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        fig.suptitle('Error Distribution', fontsize=16)
        
        for i, (col, key) in enumerate(zip(columns, column_keys)):
            errors = y_pred[:, i] - y_true[:, i]
            axes[i].hist(errors, bins=50, alpha=0.7, edgecolor='black')
            axes[i].axvline(0, color='red', linestyle='--', linewidth=2)
            axes[i].set_xlabel(f'Prediction Error ({col})')
            axes[i].set_ylabel('Frequency')
            axes[i].set_title(f'{col}\nMean Error: {np.mean(errors):.4f}')
            axes[i].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()

    def print_metrics(self, metrics):
        """評価指標をコンソールに出力"""
        print("\n" + "="*60)
        print("                モデル評価結果")
        print("="*60)
        
        for column, values in metrics.items():
            print(f"\n{column.upper()}:")
            print(f"  MAE (平均絶対誤差):      {values['MAE']:.6f}")
            print(f"  RMSE (平均平方根誤差):   {values['RMSE']:.6f}")
            print(f"  R² (決定係数):           {values['R²']:.6f}")
            print(f"  Max Error (最大誤差):    {values['Max_Error']:.6f}")
            print(f"  Correlation (相関係数):  {values['Correlation']:.6f}")
        
        print("\n" + "="*60)

def main():
    parser = argparse.ArgumentParser(description='TorchScript model evaluation')
    parser.add_argument('model_path', help='Path to trained TorchScript model (.pt file)')
    parser.add_argument('bag_path', help='Path to evaluation ROS bag directory')
    
    args = parser.parse_args()
    
    try:
        # ROS2初期化
        rclpy.init()
        
        # 評価実行
        evaluator = ModelEvaluator(args.model_path)
        
        # データ読み込み (seq_len=10でデフォルト)
        data_list = evaluator.load_rosbag_data(args.bag_path, seq_len=10)
        
        if len(data_list) == 0:
            print("エラー: データが見つかりませんでした")
            return
        
        # 入力と真値を分離
        inputs = np.array([item['input'] for item in data_list])
        targets = np.array([item['target'] for item in data_list])
        
        print("モデル推論を実行中...")
        # バッチ推論
        predictions = evaluator.predict(inputs)
        
        # 評価指標計算
        metrics = evaluator.calculate_metrics(targets, predictions)
        
        # 結果表示
        evaluator.print_metrics(metrics)
        
        # 可視化
        print("\n可視化を表示します...")
        evaluator.visualize_results(targets, predictions, metrics)
        
        print(f"\n評価完了: {len(data_list)}件のサンプルで評価しました")
        
    except Exception as e:
        print(f"エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()