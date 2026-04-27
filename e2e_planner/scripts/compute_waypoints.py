#!/usr/bin/env python3
"""
収集済みセッションデータから各フレームのウェイポイントを計算する。

data.csv の x, y, yaw (マップ座標系) を使い、
各フレームの未来軌跡を robot frame に変換して保存する。

使い方:
  python3 compute_waypoints.py --session ~/ros2_ws/dataset/session_YYYYMMDD_HHMMSS

出力:
  session/waypoints/000001.csv  ...  (x, y)
  session/data.csv に waypoints_path 列を追加 → data_with_wp.csv として保存

オプション:
  --interval    ウェイポイント間の時間間隔 [秒] (default: 0.5)
  --num-wp      ウェイポイント数 (default: 10)
"""

import argparse
import math
import sys
import numpy as np
import pandas as pd
from pathlib import Path


WAYPOINT_INTERVAL = 0.5
NUM_WAYPOINTS = 10


def transform_to_robot_frame(ref_x: float, ref_y: float, ref_yaw: float,
                              cur_x: float, cur_y: float) -> tuple:
    dx = cur_x - ref_x
    dy = cur_y - ref_y
    x_robot =  dx * math.cos(ref_yaw) + dy * math.sin(ref_yaw)
    y_robot = -dx * math.sin(ref_yaw) + dy * math.cos(ref_yaw)
    return x_robot, y_robot


def find_pose_at_time(df: pd.DataFrame, target_time: float) -> tuple:
    """target_time以降で最も近い行の (x, y, yaw) を返す。見つからなければ None。"""
    mask = df['timestamp'] >= target_time
    if not mask.any():
        return None
    row = df[mask].iloc[0]
    return (row['x'], row['y'], row['yaw'])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', required=True)
    parser.add_argument('--interval', type=float, default=WAYPOINT_INTERVAL,
                        help=f'ウェイポイント間隔[秒] (default: {WAYPOINT_INTERVAL})')
    parser.add_argument('--num-wp', type=int, default=NUM_WAYPOINTS,
                        help=f'ウェイポイント数 (default: {NUM_WAYPOINTS})')
    args = parser.parse_args()

    session_dir = Path(args.session)
    csv_path = session_dir / 'data.csv'
    if not csv_path.exists():
        print(f'エラー: {csv_path} が見つかりません')
        sys.exit(1)

    df = pd.read_csv(csv_path)
    required = {'x', 'y', 'yaw', 'timestamp'}
    if not required.issubset(df.columns):
        missing = required - set(df.columns)
        print(f'エラー: CSV に {missing} 列がありません')
        sys.exit(1)

    # NaN posを持つフレームを警告
    nan_count = df[['x', 'y', 'yaw']].isna().any(axis=1).sum()
    if nan_count > 0:
        print(f'警告: {nan_count} フレームで姿勢データがありません (NaN)')

    wp_dir = session_dir / 'waypoints'
    wp_dir.mkdir(exist_ok=True)

    waypoints_paths = []
    skipped = 0

    for _, row in df.iterrows():
        frame_id = int(row['frame_id'])
        t0 = row['timestamp']
        ref_x, ref_y, ref_yaw = row['x'], row['y'], row['yaw']

        if any(math.isnan(v) for v in [ref_x, ref_y, ref_yaw]):
            waypoints_paths.append('')
            skipped += 1
            continue

        waypoints = []
        for k in range(1, args.num_wp + 1):
            target_t = t0 + args.interval * k
            pose = find_pose_at_time(df, target_t)
            if pose is None:
                break
            wx, wy = transform_to_robot_frame(ref_x, ref_y, ref_yaw, pose[0], pose[1])
            waypoints.append((wx, wy))

        if len(waypoints) < args.num_wp:
            waypoints_paths.append('')
            skipped += 1
            continue

        wp_filename = f'{frame_id:06d}.csv'
        wp_path = wp_dir / wp_filename
        with open(wp_path, 'w', newline='') as f:
            f.write('x,y\n')
            for wx, wy in waypoints:
                f.write(f'{wx:.6f},{wy:.6f}\n')

        waypoints_paths.append(f'waypoints/{wp_filename}')

    df['waypoints_path'] = waypoints_paths
    out_csv = session_dir / 'data_with_wp.csv'
    df.to_csv(out_csv, index=False)

    valid = sum(1 for p in waypoints_paths if p)
    print(f'完了。有効フレーム: {valid}/{len(df)}  スキップ: {skipped}')
    print(f'保存先: {out_csv}')


if __name__ == '__main__':
    main()
