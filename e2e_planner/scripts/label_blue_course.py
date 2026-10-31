#!/usr/bin/env python3
"""学習データの各フレームが青コースの上を走っているかを判定し、<dataset>/course.json に書く。

青コースの基準は、bag のパスが blue/ 以下にある学習データ (info.yaml の bag) の軌跡。
次のどれかに当たるフレームを blue = 0 にする (2026-10-31、赤コースと場内を横切る道を外すため)。
  - 今の位置が基準の軌跡から D_MAX m より離れている (赤コースだけの部分、場内の道)
  - 進む向きが近くの基準と ANG_MAX 度より違う (逆走。MIN_SPEED 未満の間は向きがぶれるので見ない)
  - 10 m 先まで走った軌跡のどこかが上の 2 つに当たる
  - 正解経路の行き先 (10 m 先) が、同じ場所・同じ向きを走った青コースのフレームの行き先のどれとも
    END_TOL m 以上違う (分かれ道の手前で、経路が赤コースへまっすぐ進む。今の位置と軌跡の近さだけでは
    曲がり始めの数 m が青コースの近くに残るので見分けられない)。
    障害物を避けている場面は普通より行き先がずれる (青コース上の避ける場面で最大 3.3 m) ので、
    pylon_scenes.json で「避ける区間」と確認したフレームは END_TOL_AVOID まで許す
位置は bag の /vectornav/pose から取り直す (学習データには残っていないため)。

    python3 scripts/label_blue_course.py <dataset_dir> [<dataset_dir> ...] [--bag-root-from /media/orne/ボリューム --bag-root-to /mnt/d]
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import pymap3d as pm
import yaml
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial import cKDTree

LAT0, LON0, H0 = 36.1130995, 139.98107149999998, 0.0  # 位置の原点 (コースの近く)
D_MAX = 2.5  # 630 の bag は青コースの基準から最大 1.7 m ずれている (6/30 の GNSS)。分かれ道の手前で経路がまっすぐ進むのを外せるよう小さめに
ANG_MAX = 90.0
HORIZON = 10.0  # 経路の長さ (0.5 m x 20 点)
K_NEAR = 20
END_TOL = 2.5        # 普通に走る場面 (青コースの bag どうしで 99% が 1.8 m 以内、630 は最大 3.4 m) [m]
END_TOL_AVOID = 3.5  # 避ける区間 (分かれ道の手前の赤コースの経路は 3.1〜7 m ずれる) [m]
END_NEAR = 2.5       # 行き先を比べる青コースのフレームの範囲 [m]
END_ANG = 30.0       # 同じ向きとみなす範囲 [deg]
MIN_SPEED = 0.5  # これより遅い間は向きがぶれるので、向きは見ない [m/s]


def read_track(bag: Path) -> np.ndarray:
    """(t, east, north) の軌跡"""
    rows = []
    with AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        cons = [c for c in reader.connections if c.topic == '/vectornav/pose']
        for con, _, raw in reader.messages(connections=cons):
            msg = reader.deserialize(raw, con.msgtype)
            p = msg.pose.pose.position
            rows.append([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, p.x, p.y, p.z])
    a = np.array(rows)
    a = a[np.argsort(a[:, 0])]
    e, n, _ = pm.ecef2enu(a[:, 1], a[:, 2], a[:, 3], LAT0, LON0, H0)
    return np.c_[a[:, 0], e, n]


HEADING_WINDOW = 0.3  # 向きは前後この秒数の移動から求める (隣どうしの差だと位置のゆらぎで大きくぶれる) [s]


def heading(track: np.ndarray) -> np.ndarray:
    t = track[:, 0]
    a = [np.interp(t - HEADING_WINDOW, t, track[:, i]) for i in (1, 2)]
    b = [np.interp(t + HEADING_WINDOW, t, track[:, i]) for i in (1, 2)]
    return np.arctan2(b[1] - a[1], b[0] - a[0])


def speed(track: np.ndarray) -> np.ndarray:
    return np.hypot(np.gradient(track[:, 1], track[:, 0]), np.gradient(track[:, 2], track[:, 0]))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('datasets', type=Path, nargs='+', help='判定する学習データ。青コースの基準になる blue/ の bag も含めて渡す')
    ap.add_argument('--bag-root-from', default='/media/orne/ボリューム')
    ap.add_argument('--bag-root-to', default='/mnt/d')
    args = ap.parse_args()

    data = {}
    for d in args.datasets:
        info = yaml.safe_load(open(d / 'info.yaml'))
        if not info.get('samples'):
            continue
        bag = Path(info['bag'].replace(args.bag_root_from, args.bag_root_to))
        meta = list(csv.DictReader(open(d / 'meta.csv')))
        stems = [f"{int(r['index']):05d}" for r in meta]
        ends = np.array([np.loadtxt(d / 'path' / f'{s}.csv', delimiter=',', skiprows=1)[-1] for s in stems])
        data[d] = (info['bag'], read_track(bag), np.array([float(r['timestamp']) for r in meta]), stems, ends)

    ref = [v for v in data.values() if '/blue/' in v[0]]
    if not ref:
        raise SystemExit('青コースの基準 (bag のパスに /blue/ を含む学習データ) が 1 つも無い')
    # 基準からは止まっている間の点を除く (スタート地点で止まっている点が同じ場所に固まり、
    # 近くの点がそれだけになって向きがでたらめになる)
    ref_moving = [speed(v[1]) >= MIN_SPEED for v in ref]
    ref_xy = np.vstack([v[1][m, 1:3] for v, m in zip(ref, ref_moving)])
    ref_h = np.concatenate([heading(v[1])[m] for v, m in zip(ref, ref_moving)])
    tree = cKDTree(ref_xy)
    # 青コースのフレーム (位置・向き・行き先) の表。行き先は経路の 20 点目 (ロボット座標)
    frame_xy, frame_h, frame_end = [], [], []
    for v in ref:
        frame_xy.append(np.c_[np.interp(v[2], v[1][:, 0], v[1][:, 1]), np.interp(v[2], v[1][:, 0], v[1][:, 2])])
        frame_h.append(np.interp(v[2], v[1][:, 0], np.unwrap(heading(v[1]))))
        frame_end.append(v[4])
    frame_xy, frame_h, frame_end = np.vstack(frame_xy), np.concatenate(frame_h), np.vstack(frame_end)
    frame_tree = cKDTree(frame_xy)

    for d, (bag, track, t, stems, ends) in data.items():
        dist, near = tree.query(track[:, 1:3], k=K_NEAR)
        dang = np.degrees(np.abs((heading(track)[:, None] - ref_h[near] + np.pi) % (2 * np.pi) - np.pi)).min(axis=1)
        moving = speed(track) >= MIN_SPEED
        on_course = (dist[:, 0] <= D_MAX) & ((dang <= ANG_MAX) | ~moving)
        arc = np.r_[0.0, np.cumsum(np.hypot(np.diff(track[:, 1]), np.diff(track[:, 2])))]
        s0 = np.interp(t, track[:, 0], arc)
        k0 = np.searchsorted(track[:, 0], t)
        xy = np.c_[np.interp(t, track[:, 0], track[:, 1]), np.interp(t, track[:, 0], track[:, 2])]
        h = np.interp(t, track[:, 0], np.unwrap(heading(track)))
        scenes = d / 'pylon_scenes.json'
        avoid = {}
        if scenes.exists():
            j = json.loads(scenes.read_text())
            avoid = {s: a and r for s, a, r in zip(j['stems'], j['scene'], j['reviewed'])}
        blue = []
        for i, (k, a) in enumerate(zip(k0, s0)):
            ok = on_course[min(k, len(track) - 1)] and on_course[(arc >= a) & (arc <= a + HORIZON)].all()
            if ok:
                near = [m for m in frame_tree.query_ball_point(xy[i], END_NEAR)
                        if abs((h[i] - frame_h[m] + np.pi) % (2 * np.pi) - np.pi) < np.radians(END_ANG)]
                tol = END_TOL_AVOID if avoid.get(stems[i]) else END_TOL
                ok = bool(near) and np.linalg.norm(frame_end[near] - ends[i], axis=1).min() <= tol
            blue.append(int(ok))
        (d / 'course.json').write_text(json.dumps({'stems': stems, 'blue': blue, 'd_max': D_MAX, 'ang_max': ANG_MAX,
                                                 'end_tol': END_TOL, 'end_tol_avoid': END_TOL_AVOID}))
        print(f'{d.name:34s} 青コース {sum(blue):5d} / {len(blue):5d}', flush=True)


if __name__ == '__main__':
    main()
