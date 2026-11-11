#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

import matplotlib


def load(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    data = {k: [float(r[k]) for r in rows] for k in ['s', 'e_y', 'e_psi', 'v', 'v_ref', 'plan_offset', 'steering', 'x', 'y', 'course_x', 'course_y']}
    curve = [float(r['s']) for r in rows if r['section'] == 'curve']
    data['curve'] = (min(curve), max(curve)) if curve else None
    return data


def plot(paths, output, show=False):
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(16, 10))
    grid = fig.add_gridspec(4, 2, width_ratios=[1, 1.4])
    ax_traj = fig.add_subplot(grid[:, 0])
    axes = [fig.add_subplot(grid[0, 1])]
    axes += [fig.add_subplot(grid[i, 1], sharex=axes[0]) for i in range(1, 4)]
    ax_ey, ax_epsi, ax_v, ax_steer = axes

    for n, path in enumerate(paths):
        d = load(path)
        color = f'C{n}'
        label = Path(path).stem
        if n == 0:
            ax_traj.plot(d['course_x'], d['course_y'], 'k--', lw=1, label='course')
            if d['curve']:
                for ax in axes:
                    ax.axvspan(*d['curve'], color='0.9')
        ax_traj.plot(d['x'], d['y'], color=color, label=label)
        ax_ey.plot(d['s'], d['e_y'], color=color)
        ax_ey.plot(d['s'], d['plan_offset'], color=color, ls=':')
        ax_epsi.plot(d['s'], [math.degrees(e) for e in d['e_psi']], color=color)
        ax_v.plot(d['s'], d['v'], color=color)
        ax_v.plot(d['s'], d['v_ref'], color=color, ls='--')
        ax_steer.plot(d['s'], [math.degrees(e) for e in d['steering']], color=color)

    ax_traj.set_aspect('equal')
    ax_traj.set_xlabel('x [m]')
    ax_traj.set_ylabel('y [m]')
    ax_traj.legend(fontsize=8)
    ax_ey.set_ylabel('e_y (solid)\nplan_offset (dotted) [m]')
    ax_epsi.set_ylabel('heading err [deg]')
    ax_v.set_ylabel('v (solid)\nv_ref (dashed) [m/s]')
    for limit in (15.0, -15.0):
        ax_steer.axhline(limit, color='r', lw=0.8)
    ax_steer.set_ylabel('steering [deg]')
    ax_steer.set_xlabel('s [m]')
    for ax in [ax_traj] + axes:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(output, dpi=120)
    print(f'{output} に保存しました')
    if show:
        plt.show()
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv', nargs='+')
    parser.add_argument('-o', '--output')
    parser.add_argument('--show', action='store_true')
    args = parser.parse_args()
    output = args.output or (str(Path(args.csv[0]).with_suffix('.png')) if len(args.csv) == 1 else 'course_replay_compare.png')
    plot(args.csv, output, args.show)


if __name__ == '__main__':
    main()
