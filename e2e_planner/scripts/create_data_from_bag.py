#!/usr/bin/env python3
"""rosbag から学習データ (images/ path/ meta.csv info.yaml) をオフラインで作る。

e2e_train_20260928_rgb_mask を作った変換スクリプトが残っていなかったため、
その datasets.yaml の設定値と出来上がったデータに合わせて作り直したもの (2026-10-30)。
経路のラベルは create_data.py と同じ util/waypoints.py で作る。

  - サンプル: /zed/zed_node/rgb/image_rect_color の header.stamp で SAMPLE_INTERVAL 以上空いた画像
  - 経路: /vectornav/pose (ECEF + ヨー) から 0.5 m 間隔 x 20 点。ヨーは補正しない
    (yaw_bias_deg は測って info.yaml に残すだけ。元のデータもそうなっている)
  - 落とす条件 (dropped_rejected): 速度 < MIN_SPEED、10 m 先に着いたときの速度 < MIN_SPEED
    (止まりかけ)、10 m 先まで届かない、区間内の pose の間隔 > MAX_POSE_GAP、
    区間内の pose の跳び > MAX_JUMP_SPEED、|ヨーと進行方向の差| > MAX_YAW_OFFSET
  - 速度とヨーの差は前後 SPEED_WINDOW 秒の移動から求める。元のスクリプトの定義は分からず、
    元のデータとの差は速度 0.05 m/s・ヨー 0.6 度以内 (16_44_53 で確認)
白線マスク (mask_images/) はここでは作らない。

    python3 scripts/create_data_from_bag.py <bag> <out_root> [--name NAME]
    → <out_root>/<NAME (既定は bag のディレクトリ名)>/{images,path}/<00001>.png|csv, meta.csv, info.yaml
"""

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
import pymap3d as pm
import yaml
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from util.waypoints import PoseSample, build_forward_arc, interpolate_pose, pose_at_distance, transform_to_robot_frame

IMAGE_TOPIC = '/zed/zed_node/rgb/image_rect_color'
POSE_TOPIC = '/vectornav/pose'
SAMPLE_INTERVAL = 0.2
WAYPOINT_DISTANCE = 0.5
NUM_WAYPOINTS = 20
MIN_SPEED = 0.5
MAX_POSE_GAP = 0.3
MAX_JUMP_SPEED = 8.0
MAX_YAW_OFFSET = 4.0
SPEED_WINDOW = 0.5
YAW_SUSPECT_DEG = 2.0  # |yaw_bias_deg| がこれを超えたら yaw_suspect (元のデータ: 0.81 は false、2.77 は true)


def yaw_of(q) -> float:
    x, y, z, w = q
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def read_bag(bag: Path):
    """pose 列と、SAMPLE_INTERVAL ごとの画像 (BGR) を返す"""
    poses, samples, last = [], [], None
    with AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        cons = [c for c in reader.connections if c.topic in (IMAGE_TOPIC, POSE_TOPIC)]
        for con, _, raw in reader.messages(connections=cons):
            msg = reader.deserialize(raw, con.msgtype)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if con.topic == POSE_TOPIC:
                if poses and t <= poses[-1].timestamp:
                    continue  # 同時刻/逆順は補間を壊すので捨てる (create_data.py と同じ)
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                poses.append(PoseSample(t, np.array([p.x, p.y, p.z]), yaw_of([q.x, q.y, q.z, q.w])))
            elif last is None or t - last >= SAMPLE_INTERVAL:
                last = t
                ch = 4 if msg.encoding in ('bgra8', 'rgba8') else 3
                img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch]
                img = img.reshape(msg.height, msg.width, ch)
                if msg.encoding == 'bgra8':
                    img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                elif msg.encoding == 'rgba8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
                elif msg.encoding == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                samples.append((t, img.copy()))
    return poses, samples


def convert(bag: Path, out_dir: Path) -> dict:
    poses, samples = read_bag(bag)
    pt = np.array([p.timestamp for p in poses])
    pos = np.array([p.position for p in poses])
    step = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    arc = np.r_[0.0, np.cumsum(step)]
    step_speed = np.r_[0.0, step / np.maximum(np.diff(pt), 1e-6)]
    # ロボットの前方は ENU でヨー + 90 度 (transform_to_robot_frame の定義)
    heading = np.unwrap([p.yaw for p in poses]) + np.pi / 2

    def at(t, arr):
        return np.interp(t, pt, arr)

    def speed(t):
        a, b = max(t - SPEED_WINDOW, pt[0]), min(t + SPEED_WINDOW, pt[-1])
        return (at(b, arc) - at(a, arc)) / max(b - a, 1e-6)

    def yaw_offset_deg(t):
        a, b = max(t - SPEED_WINDOW, pt[0]), min(t + SPEED_WINDOW, pt[-1])
        pa = [at(a, pos[:, i]) for i in range(3)]
        pb = [at(b, pos[:, i]) for i in range(3)]
        lat, lon, h = pm.ecef2geodetic(*pa)
        e, n, _ = pm.ecef2enu(*pb, lat, lon, h)
        d = np.arctan2(n, e) - at(t, heading)
        return float(np.degrees((d + np.pi) % (2 * np.pi) - np.pi))

    rows, offsets, no_pose, rejected, reasons = [], [], 0, 0, []
    for t, img in samples:
        ref = interpolate_pose(poses, t)
        if ref is None:
            no_pose += 1
            continue
        s0 = at(t, arc)
        v = speed(t)
        off = yaw_offset_deg(t)
        if v >= MIN_SPEED:
            offsets.append(off)
        horizon = WAYPOINT_DISTANCE * NUM_WAYPOINTS
        why = []
        if v < MIN_SPEED:
            why.append('slow')
        if arc[-1] - s0 < horizon:
            why.append('no_horizon')
        if abs(off) > MAX_YAW_OFFSET:
            why.append('yaw_offset')
        if not why:
            t_end = float(np.interp(s0 + horizon, arc, pt))
            idx = np.flatnonzero((pt >= t) & (pt <= t_end))
            lo, hi = max(idx[0] - 1, 0) if len(idx) else 0, min(idx[-1] + 1, len(pt) - 1) if len(idx) else 0
            if speed(t_end) < MIN_SPEED:
                why.append('stops_in_horizon')
            if np.diff(pt[lo:hi + 1]).max(initial=0.0) > MAX_POSE_GAP:
                why.append('pose_gap')
            if step_speed[lo + 1:hi + 1].max(initial=0.0) > MAX_JUMP_SPEED:
                why.append('pose_jump')
        if why:
            rejected += 1
            reasons.append((t, s0, v, off, '+'.join(why)))
            continue
        fwd = build_forward_arc(poses, ref)
        wps = [transform_to_robot_frame(ref, pose_at_distance(fwd, WAYPOINT_DISTANCE * (i + 1))) for i in range(NUM_WAYPOINTS)]
        rows.append((t, img, wps, s0, v, off))

    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / 'images').mkdir(exist_ok=True)
    (out_dir / 'path').mkdir(exist_ok=True)
    with open(out_dir / 'meta.csv', 'w', newline='') as fm:
        mw = csv.writer(fm)
        mw.writerow(['index', 'timestamp', 'arc_m', 'speed_mps', 'yaw_offset_deg'])
        for k, (t, img, wps, s0, v, off) in enumerate(rows, start=1):
            cv2.imwrite(str(out_dir / 'images' / f'{k:05d}.png'), img)
            with open(out_dir / 'path' / f'{k:05d}.csv', 'w', newline='') as fp:
                w = csv.writer(fp)
                w.writerow(['x', 'y'])
                w.writerows(wps)
            mw.writerow([k, f'{t:.6f}', f'{s0:.4f}', f'{v:.4f}', f'{off:.3f}'])

    # 落としたフレームと理由 (どの条件で何枚落ちたかを後から確かめるため)
    with open(out_dir / 'rejected.csv', 'w', newline='') as fr:
        rw = csv.writer(fr)
        rw.writerow(['timestamp', 'arc_m', 'speed_mps', 'yaw_offset_deg', 'reason'])
        rw.writerows((f'{t:.6f}', f'{s0:.4f}', f'{v:.4f}', f'{off:.3f}', why) for t, s0, v, off, why in reasons)

    bias = float(np.median(offsets)) if offsets else 0.0
    W = np.array([r[2] for r in rows]) if rows else np.zeros((0, NUM_WAYPOINTS, 2))
    sp = np.array([r[4] for r in rows]) if rows else np.zeros(0)
    info = {
        'bag': str(bag), 'samples': len(rows), 'candidates': len(samples),
        'dropped_no_pose': no_pose, 'dropped_rejected': rejected,
        'min_speed': MIN_SPEED, 'max_pose_gap': MAX_POSE_GAP, 'max_jump_speed': MAX_JUMP_SPEED,
        'max_yaw_offset': MAX_YAW_OFFSET, 'yaw_bias_deg': round(bias, 3),
        'yaw_suspect': bool(abs(bias) > YAW_SUSPECT_DEG), 'require_moving_horizon': True,
        'waypoint_mode': 'distance', 'waypoint_distance': WAYPOINT_DISTANCE, 'num_waypoints': NUM_WAYPOINTS,
        'speed_mean': float(sp.mean()) if len(sp) else 0.0, 'speed_max': float(sp.max()) if len(sp) else 0.0,
        'waypoint_x_min': float(W[..., 0].min()) if len(W) else 0.0, 'waypoint_x_max': float(W[..., 0].max()) if len(W) else 0.0,
        'waypoint_y_min': float(W[..., 1].min()) if len(W) else 0.0, 'waypoint_y_max': float(W[..., 1].max()) if len(W) else 0.0,
        'rejected_by': {k: sum(k in r[4].split('+') for r in reasons)
                        for k in ('slow', 'no_horizon', 'yaw_offset', 'stops_in_horizon', 'pose_gap', 'pose_jump')},
        'converter': 'create_data_from_bag.py',
    }
    with open(out_dir / 'info.yaml', 'w') as f:
        yaml.safe_dump(info, f, allow_unicode=True, sort_keys=False)
    return info


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', type=Path)
    ap.add_argument('out_root', type=Path)
    ap.add_argument('--name', default=None, help='出力ディレクトリ名 (既定は bag のディレクトリ名)')
    args = ap.parse_args()
    info = convert(args.bag, args.out_root / (args.name or args.bag.name))
    print({k: info[k] for k in ('samples', 'candidates', 'dropped_no_pose', 'dropped_rejected', 'yaw_bias_deg', 'yaw_suspect')})


if __name__ == '__main__':
    main()
