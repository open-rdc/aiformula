#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
from collections import deque
from datetime import datetime


class PathPlotter(Node):
    def __init__(self):
        super().__init__('path_plotter')
        
        # 前回のパス
        self.previous_path = None
        
        # 連結された軌跡データ（ワールド座標）
        self.trajectory_points = []
        self.trajectory_timestamps = []
        
        # 現在のオフセット（累積的な位置）
        self.current_offset = np.array([0.0, 0.0])
        
        # サブスクライバー
        self.sub_target_path = self.create_subscription(
            Path,
            'e2e_planner/path',
            self.target_path_callback,
            10
        )
        
        self.get_logger().info('Path Plotter initialized')
        self.get_logger().info('Subscribing to: e2e_planner/path')
        self.get_logger().info('Accumulating trajectory by connecting paths...')
        self.get_logger().info('Press Ctrl+C to stop and display trajectory')
        
    def target_path_callback(self, msg: Path):
        """受信したパスを前回のパスに繋げる"""
        if len(msg.poses) == 0:
            return
        
        # 現在のパスを配列に変換
        current_path = np.array([(pose.pose.position.x, pose.pose.position.y) 
                                for pose in msg.poses])
        
        if self.previous_path is None:
            # 最初のパス：そのままオフセット0で保存
            for point in current_path:
                self.trajectory_points.append(tuple(point))
                self.trajectory_timestamps.append(self.get_clock().now().nanoseconds / 1e9)
        else:
            # 前回のパスと今回のパスからロボットの移動量を推定
            # 戦略: 前回のパスのある点が、今回のパスではマイナス方向に移動している
            # 例: 前回のパスの[2.0, 0]が、今回のパスでは見えなくなった
            #     = ロボットが2m前進した
            
            # 前回のパスの先頭数点と、今回のパスを比較
            # 前回のパスの第i点が、今回のパスの原点付近に来ているはず
            
            # シンプルな仮定：前回のパスの1番目の点 ≈ 今回のロボット位置（原点）まで移動
            # つまり、previous_path[1]が今のbase_link原点に来た = その距離だけ移動
            
            # より正確には、前回のパスのどの点が今回の原点に最も近いかを見つける
            min_dist = float('inf')
            best_idx = 0
            
            for i in range(min(len(self.previous_path), 5)):  # 最初の5点だけチェック
                dist = np.linalg.norm(self.previous_path[i] - current_path[0])
                if dist < min_dist:
                    min_dist = dist
                    best_idx = i
            
            # ロボットの移動量 = 前回のパスのbest_idx番目の点の位置
            # （なぜなら、その点が今は原点に来ているから）
            if best_idx > 0 and best_idx < len(self.previous_path):
                movement = self.previous_path[best_idx]
            else:
                # フォールバック: パスの長さの変化から推定
                # または、デフォルトで小さな前進を仮定
                movement = np.array([0.1, 0.0])
            
            # オフセットを更新
            self.current_offset += movement
            
            # 現在のパスをオフセット分ずらして軌跡に追加
            for i, point in enumerate(current_path):
                if i > 0 or len(self.trajectory_points) == 0:
                    world_point = point + self.current_offset
                    
                    # 重複チェック
                    if len(self.trajectory_points) == 0:
                        self.trajectory_points.append(tuple(world_point))
                        self.trajectory_timestamps.append(self.get_clock().now().nanoseconds / 1e9)
                    else:
                        last_point = np.array(self.trajectory_points[-1])
                        distance = np.linalg.norm(world_point - last_point)
                        if distance > 0.01:
                            self.trajectory_points.append(tuple(world_point))
                            self.trajectory_timestamps.append(self.get_clock().now().nanoseconds / 1e9)
        
        # 現在のパスを保存
        self.previous_path = current_path.copy()
        
        self.get_logger().info(
            f'Trajectory points: {len(self.trajectory_points)}', 
            throttle_duration_sec=1.0
        )
    
    def plot_results(self):
        """蓄積された軌跡をプロット"""
        if len(self.trajectory_points) == 0:
            self.get_logger().warning('No trajectory data received')
            return
        
        self.get_logger().info(f'Plotting trajectory with {len(self.trajectory_points)} points...')
        
        plt.figure(figsize=(12, 10))
        
        # 軌跡データを展開
        x_vals = [x for x, y in self.trajectory_points]
        y_vals = [y for x, y in self.trajectory_points]
        
        # 時系列でカラーマップを作成
        if len(self.trajectory_timestamps) > 1:
            time_elapsed = np.array(self.trajectory_timestamps) - self.trajectory_timestamps[0]
            
            # 時間に応じた色でプロット
            points = np.array([x_vals, y_vals]).T.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)
            
            from matplotlib.collections import LineCollection
            lc = LineCollection(segments, cmap='rainbow', linewidth=2)
            lc.set_array(time_elapsed[:-1])
            
            ax = plt.gca()
            line = ax.add_collection(lc)
            
            # 軸の範囲を設定
            margin = 2.0
            ax.set_xlim(min(x_vals) - margin, max(x_vals) + margin)
            ax.set_ylim(min(y_vals) - margin, max(y_vals) + margin)
            
            # カラーバーを追加
            cbar = plt.colorbar(line, ax=ax, orientation='vertical', pad=0.02)
            cbar.set_label('Time (s)', rotation=270, labelpad=20)
        else:
            plt.plot(x_vals, y_vals, 'b-', linewidth=2, label='Trajectory')
        
        # 始点と終点を強調
        plt.plot(x_vals[0], y_vals[0], 'go', markersize=12, label='Start', zorder=5)
        plt.plot(x_vals[-1], y_vals[-1], 'rx', markersize=12, label='End', zorder=5)
        
        plt.title(f'Robot Trajectory (Path-based)\n({len(self.trajectory_points)} points)')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.legend(loc='best')
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        # 統計情報を表示
        total_duration = 0
        total_distance = 0
        
        if len(self.trajectory_timestamps) > 1:
            total_duration = self.trajectory_timestamps[-1] - self.trajectory_timestamps[0]
        
        # 走行距離を計算
        for i in range(1, len(self.trajectory_points)):
            dx = x_vals[i] - x_vals[i-1]
            dy = y_vals[i] - y_vals[i-1]
            total_distance += np.sqrt(dx**2 + dy**2)
        
        avg_speed = total_distance / total_duration if total_duration > 0 else 0
        
        info_text = (
            f'Points: {len(self.trajectory_points)}\n'
            f'Duration: {total_duration:.2f} s\n'
            f'Distance: {total_distance:.2f} m\n'
            f'Avg speed: {avg_speed:.2f} m/s'
        )
        
        plt.text(0.02, 0.98, info_text, transform=plt.gca().transAxes,
                fontsize=10, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        plt.tight_layout()
        plt.show()
        
        # コンソールに統計情報を出力
        self.get_logger().info('=== Trajectory Statistics ===')
        self.get_logger().info(f'Total points: {len(self.trajectory_points)}')
        self.get_logger().info(f'Total duration: {total_duration:.2f} s')
        self.get_logger().info(f'Total distance: {total_distance:.2f} m')
        self.get_logger().info(f'Average speed: {avg_speed:.2f} m/s')


def main(args=None):
    rclpy.init(args=args)
    node = PathPlotter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        # 蓄積した軌跡をプロット表示
        node.plot_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()