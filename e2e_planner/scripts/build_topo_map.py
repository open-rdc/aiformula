#!/usr/bin/env python3
"""
セッションデータからトポロジカルマップを生成する。

使い方:
  python3 build_topo_map.py --session ~/ros2_ws/dataset/session_YYYYMMDD_HHMMSS

出力:
  session/topo_map.json

マップ構造:
  keyframes: 位置・ラベル・画像パスを持つキーフレームのリスト
  route: ナビゲーション時のキーフレーム順序
  decision_points: 分岐判断が必要なキーフレームIDのリスト
"""

import argparse
import json
import math
import sys
import numpy as np
import pandas as pd
from pathlib import Path


KEYFRAME_DIST = 1.0   # キーフレーム間の最小距離 [m]
DECISION_LOOKAHEAD = 4.0  # 分岐点の何m手前からコマンドを出すか [m]


def dist2d(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def build_keyframes(df: pd.DataFrame, min_dist: float) -> list:
    """走行軌跡からキーフレームを等間隔で抽出する"""
    keyframes = []
    last_pos = None

    for _, row in df.iterrows():
        pos = (row['x'], row['y'], row['yaw'])
        if last_pos is None or dist2d(pos, last_pos) >= min_dist:
            kf = {
                'id': len(keyframes),
                'image': str(row.get('image', '')),
                'mask': str(row.get('mask', '')),
                'x': float(row['x']),
                'y': float(row['y']),
                'yaw': float(row['yaw']),
                'label': int(row['label']),
                'label_name': str(row.get('label_name', '')),
            }
            keyframes.append(kf)
            last_pos = pos

    return keyframes


def find_decision_points(keyframes: list) -> list:
    """
    ラベルが道なり(0)以外のキーフレームを分岐点として検出する。
    連続したラベル変化を一つの分岐点としてまとめる。
    """
    decision_points = []
    prev_label = 0
    in_decision = False

    for kf in keyframes:
        label = kf['label']
        if label != 0 and not in_decision:
            decision_points.append(kf['id'])
            in_decision = True
        elif label == 0:
            in_decision = False
        prev_label = label

    return decision_points


def compute_command_schedule(keyframes: list, lookahead_dist: float) -> dict:
    """
    各キーフレームでのナビゲーションコマンドを計算する。
    分岐点のlookahead_dist手前からコマンドを出し始める。
    """
    n = len(keyframes)
    commands = {kf['id']: 0 for kf in keyframes}  # デフォルト: 道なり

    # 分岐点ごとに、手前のキーフレームにも同じコマンドを設定
    for i, kf in enumerate(keyframes):
        if kf['label'] != 0:
            # この分岐点から後ろlookahead_dist以内のキーフレームにコマンドを伝播
            for j in range(i - 1, -1, -1):
                d = dist2d(
                    (keyframes[j]['x'], keyframes[j]['y']),
                    (kf['x'], kf['y'])
                )
                if d <= lookahead_dist:
                    commands[keyframes[j]['id']] = kf['label']
                else:
                    break
            # 分岐点自体と、直後のキーフレームにも設定
            commands[kf['id']] = kf['label']
            if i + 1 < n:
                commands[keyframes[i + 1]['id']] = kf['label']

    return commands


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', required=True, help='セッションディレクトリのパス')
    parser.add_argument('--keyframe-dist', type=float, default=KEYFRAME_DIST,
                        help=f'キーフレーム間距離[m] (デフォルト: {KEYFRAME_DIST})')
    parser.add_argument('--lookahead', type=float, default=DECISION_LOOKAHEAD,
                        help=f'分岐事前通知距離[m] (デフォルト: {DECISION_LOOKAHEAD})')
    args = parser.parse_args()

    session_dir = Path(args.session)

    # 二値化済みCSVを優先、なければ元のCSVを使用
    csv_path = session_dir / 'data_binarized.csv'
    if not csv_path.exists():
        csv_path = session_dir / 'data.csv'
        print(f'警告: data_binarized.csv が見つかりません。{csv_path} を使用します')
        print('  binarize_dataset.py を先に実行することを推奨します')

    if not csv_path.exists():
        print(f'エラー: {csv_path} が見つかりません')
        sys.exit(1)

    df = pd.read_csv(csv_path)
    print(f'{len(df)} フレームを読み込みました')

    # キーフレーム抽出
    keyframes = build_keyframes(df, args.keyframe_dist)
    print(f'キーフレーム数: {len(keyframes)} (間隔: {args.keyframe_dist}m)')

    # 分岐点検出
    decision_point_ids = find_decision_points(keyframes)
    label_counts = df['label'].value_counts().to_dict()
    print(f'分岐点数: {len(decision_point_ids)}')
    print(f'ラベル分布: {label_counts}')

    # コマンドスケジュール計算
    commands = compute_command_schedule(keyframes, args.lookahead)
    for kf in keyframes:
        kf['nav_command'] = commands[kf['id']]

    # ルート = キーフレームIDの順序リスト
    route = [kf['id'] for kf in keyframes]

    # 統計情報
    total_dist = sum(
        dist2d((keyframes[i]['x'], keyframes[i]['y']),
                (keyframes[i-1]['x'], keyframes[i-1]['y']))
        for i in range(1, len(keyframes))
    )

    topo_map = {
        'metadata': {
            'session': str(session_dir),
            'keyframe_distance': args.keyframe_dist,
            'decision_lookahead': args.lookahead,
            'num_keyframes': len(keyframes),
            'num_decision_points': len(decision_point_ids),
            'total_distance_m': round(total_dist, 2),
            'labels': {'0': 'straight', '1': 'right', '2': 'left'},
        },
        'keyframes': keyframes,
        'route': route,
        'decision_points': decision_point_ids,
    }

    out_path = session_dir / 'topo_map.json'
    with open(out_path, 'w') as f:
        json.dump(topo_map, f, indent=2, ensure_ascii=False)

    print(f'\nトポロジカルマップを保存: {out_path}')
    print(f'  総走行距離: {total_dist:.2f} m')
    print(f'  分岐点: {[keyframes[i]["label_name"] for i in decision_point_ids[:5]]}...')


if __name__ == '__main__':
    main()
