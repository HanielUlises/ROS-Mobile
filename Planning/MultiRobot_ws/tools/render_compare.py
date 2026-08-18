#!/usr/bin/env python3
"""Compare two recorded runs of the same world under different policies.

The third iteration puts a deliberative frontier planner beside the reactive
wall follower of the first two, in the same building, from the same deployment
poses, under the same link process. The comparison is only worth anything if
both runs are read the same way, so this tool takes both run directories and
emits:

    fig_policies.png   final fused grid and trajectories, one panel per policy
    fig_compare.png    explored area and redundancy against time, both policies
    fig_planner.png    what the planner was deciding: frontiers, committed
                       travel, and the link state that gates both

Runs are truncated to a common simulation-time window, since two runs of the
same wall-clock length do not reach the same simulation time.

    python3 tools/render_compare.py \\
        reactive=/tmp/mrs_iter3_reactive frontier=/tmp/mrs_iter3_frontier \\
        -o docs/figures/iter3 --reachable 303.1
"""

import argparse
import csv
import glob
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mrs_style as style  # noqa: E402
import render_run as rr  # noqa: E402

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.ticker import AutoMinorLocator  # noqa: E402


# --------------------------------------------------------------------------
# run ingest
# --------------------------------------------------------------------------

def robot_names(row):
    return sorted({m.group(1) for m in (re.match(r'(robot\d+)_area_m2', k) for k in row)
                   if m})


def load_run(path, until=None):
    """Read one run directory into a dictionary of time series and grids."""
    with open(os.path.join(path, 'coverage.csv')) as handle:
        rows = [r for r in csv.DictReader(handle) if float(r['sim_time']) > 0.0]
    if until is not None:
        rows = [r for r in rows if float(r['sim_time']) <= until]
    if not rows:
        raise SystemExit(f'{path}: no samples')

    names = robot_names(rows[0])
    run = dict(
        path=path,
        names=names,
        t=np.array([float(r['sim_time']) for r in rows]),
        fused=np.array([float(r['merged_area_m2']) for r in rows]),
        agent={n: np.array([float(r[f'{n}_area_m2']) for r in rows]) for n in names},
        linked={n: np.array([int(r[f'{n}_linked']) for r in rows]) for n in names},
    )

    # Trajectories: the estimate, with the samples before an agent's SLAM
    # instance has published anything left out rather than filled in.
    run['xy'] = {}
    for name in names:
        pts = [(float(r[f'{name}_x']), float(r[f'{name}_y'])) for r in rows
               if r[f'{name}_x'] not in (None, '')]
        run['xy'][name] = np.array(pts) if pts else np.empty((0, 2))

    run['frame'] = int(rows[-1]['frame'])
    fused_path = os.path.join(path, f"frame_{run['frame']:05d}.grid")
    run['grid'] = rr.read_grid(fused_path)

    run['planner'] = {}
    for name in names:
        log = os.path.join(path, f'planner_{name}.csv')
        if not os.path.exists(log):
            continue
        with open(log) as handle:
            plan_rows = [r for r in csv.DictReader(handle)]
        if until is not None:
            plan_rows = [r for r in plan_rows if float(r['sim_time']) <= until]
        run['planner'][name] = plan_rows
    return run


def path_length(points):
    if len(points) < 2:
        return 0.0
    return float(np.hypot(*np.diff(points, axis=0).T).sum())


# --------------------------------------------------------------------------
# figures
# --------------------------------------------------------------------------

def figure_policies(runs, labels, out_path):
    """The two final fused grids, with the trajectories that produced them."""
    metas = [runs[k]['grid'][1] for k in labels]
    canvas = rr.common_canvas(metas)
    extent = rr.extent_of(canvas)
    aspect = (extent[3] - extent[2]) / (extent[1] - extent[0])

    width = 6.9
    fig, axes = plt.subplots(len(labels), 1,
                             figsize=(width, width * aspect * len(labels) + 1.0),
                             gridspec_kw=dict(hspace=0.35))

    for index, (ax, key) in enumerate(zip(np.atleast_1d(axes), labels)):
        run = runs[key]
        cells, meta = run['grid']
        ax.imshow(rr.composite(cells, meta, canvas), cmap=style.OCC_CMAP,
                  norm=style.OCC_NORM, origin='lower', extent=extent,
                  interpolation='nearest', rasterized=True)
        for j, name in enumerate(run['names']):
            xy = run['xy'][name]
            if len(xy) == 0:
                continue
            colour = style.SERIES[j % len(style.SERIES)]
            ax.plot(xy[:, 0], xy[:, 1], lw=0.9, color=colour, zorder=4,
                    label=rf'agent {name[-1]}')
            ax.plot([xy[-1, 0]], [xy[-1, 1]], marker='o', ms=3.5, color=colour,
                    zorder=5, linestyle='none')
        rr.dress_map_axes(ax, extent)
        ax.set_title(f'{key} policy: '
                     rf'$M$ = {run["fused"][-1]:.1f} m$^2$ after {run["t"][-1]:.0f} s')
        style.panel_label(ax, '(a)' if index == 0 else '(b)')
        if index == 0:
            ax.legend(loc='upper right', ncol=3, handlelength=1.2, framealpha=0.9)

    fig.savefig(out_path)
    plt.close(fig)
    print(f'wrote {out_path}')


def figure_compare(runs, labels, out_path, reachable=None):
    """Explored area and observation redundancy against time, both policies."""
    fig, axes = plt.subplots(1, 2, figsize=(6.9, 2.6),
                             gridspec_kw=dict(wspace=0.28))

    styles = {labels[0]: dict(color=style.SERIES[3], ls='--'),
              labels[1]: dict(color=style.INK, ls='-')}

    ax = axes[0]
    for key in labels:
        run = runs[key]
        ax.plot(run['t'], run['fused'], lw=1.4, label=f'{key}, fused $M$',
                **styles[key])
        for j, name in enumerate(run['names']):
            ax.plot(run['t'], run['agent'][name], lw=0.5, alpha=0.55,
                    color=styles[key]['color'], ls=styles[key]['ls'])
    if reachable:
        ax.axhline(reachable, lw=0.8, color=style.RULE, ls=':')
        ax.annotate('reachable floor', xy=(runs[labels[0]]['t'][-1] * 0.02,
                                           reachable),
                    xytext=(0, -9), textcoords='offset points',
                    fontsize=6.5, color=style.MUTED)
    ax.set_xlabel(r'simulation time [s]')
    ax.set_ylabel(r'explored area [m$^2$]')
    ax.legend(loc='lower right')
    style.panel_label(ax, '(a)')

    # Redundancy: how much of what the agents individually observed was ground
    # somebody else had already observed. One is a perfect partition of the
    # work; n is n agents doing the same work n times.
    ax = axes[1]
    for key in labels:
        run = runs[key]
        total = sum(run['agent'][n] for n in run['names'])
        with np.errstate(divide='ignore', invalid='ignore'):
            ratio = np.where(run['fused'] > 1.0, total / run['fused'], np.nan)
        ax.plot(run['t'], ratio, lw=1.4, label=key, **styles[key])
    ax.axhline(1.0, lw=0.8, color=style.RULE, ls=':')
    ax.set_xlabel(r'simulation time [s]')
    ax.set_ylabel(r'$\sum_i A(m_i) \,/\, A(M)$')
    ax.legend(loc='upper left')
    style.panel_label(ax, '(b)')

    for ax in axes:
        ax.xaxis.set_minor_locator(AutoMinorLocator(2))
        ax.yaxis.set_minor_locator(AutoMinorLocator(2))

    fig.savefig(out_path)
    plt.close(fig)
    print(f'wrote {out_path}')


def figure_planner(run, out_path):
    """What the planner had to work with, and what it committed to."""
    names = sorted(run['planner'])
    if not names:
        print('no planner logs, skipping planner figure')
        return

    fig, axes = plt.subplots(2, 1, figsize=(6.9, 3.6), sharex=True,
                             gridspec_kw=dict(hspace=0.25, height_ratios=[1, 1]))

    ax = axes[0]
    for j, name in enumerate(names):
        rows = run['planner'][name]
        t = np.array([float(r['sim_time']) for r in rows])
        n = np.array([int(r['n_frontiers']) for r in rows])
        ax.plot(t, n, lw=1.0, color=style.SERIES[j % len(style.SERIES)],
                label=rf'agent {name[-1]}')
    ax.set_ylabel('reachable\nfrontiers')
    ax.legend(loc='upper right', ncol=3, handlelength=1.2)
    style.panel_label(ax, '(a)')

    ax = axes[1]
    for j, name in enumerate(names):
        rows = run['planner'][name]
        t = np.array([float(r['sim_time']) for r in rows])
        d = np.array([float(r['goal_distance'] or 'nan') for r in rows])
        d[~np.isfinite(d)] = np.nan
        colour = style.SERIES[j % len(style.SERIES)]
        ax.plot(t, d, lw=1.0, color=colour)

        # Outages of this agent's link, from the planner's own view of it.
        linked = np.array([int(r['linked']) for r in rows])
        for start, end in rr.outage_spans(t, linked):
            ax.axvspan(start, end, ymin=0.02 + 0.05 * j, ymax=0.06 + 0.05 * j,
                       color=colour, alpha=0.35, lw=0)
    ax.set_ylabel('committed\ntravel [m]')
    ax.set_xlabel(r'simulation time [s]')
    style.panel_label(ax, '(b)')

    for ax in axes:
        ax.xaxis.set_minor_locator(AutoMinorLocator(2))
        ax.yaxis.set_minor_locator(AutoMinorLocator(2))

    fig.savefig(out_path)
    plt.close(fig)
    print(f'wrote {out_path}')


# --------------------------------------------------------------------------

def summarise(runs, labels, reachable=None):
    for key in labels:
        run = runs[key]
        total = sum(run['agent'][n][-1] for n in run['names'])
        print(f'\n[{key}] {run["path"]}  t = {run["t"][-1]:.0f} s')
        print(f'  fused area          {run["fused"][-1]:8.1f} m2'
              + (f'  ({100 * run["fused"][-1] / reachable:.1f} % of reachable)'
                 if reachable else ''))
        for name in run['names']:
            share = 100 * run['agent'][name][-1] / run['fused'][-1]
            down = 100 * (1 - run['linked'][name].mean())
            print(f'  {name}: {run["agent"][name][-1]:8.1f} m2  '
                  f'{share:5.1f} % of M   downtime {down:4.1f} %   '
                  f'path {path_length(run["xy"][name]):6.1f} m')
        print(f'  redundancy          {total / run["fused"][-1]:8.3f}'
              f'   ({total - run["fused"][-1]:.1f} m2 duplicated)')
        # 90 % / 95 % of this run's own final extent, as a saturation measure.
        for frac in (0.9, 0.95):
            hit = np.nonzero(run['fused'] >= frac * run['fused'][-1])[0]
            if len(hit):
                print(f'  {100 * frac:.0f} % of final extent at '
                      f'{run["t"][hit[0]]:.0f} s')
        for name in sorted(run['planner']):
            last = run['planner'][name][-1]
            print(f'  {name} planner: {last["replans"]} replans, '
                  f'{last["reached"]} frontiers reached, '
                  f'{last["abandoned"]} abandoned, '
                  f'{last["deliveries"]} fleet maps delivered, '
                  f'{last["withheld"]} withheld')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('runs', nargs=2, metavar='LABEL=DIR',
                        help='the two runs to compare, baseline first')
    parser.add_argument('-o', '--output', required=True, help='figure directory')
    parser.add_argument('--reachable', type=float, default=None,
                        help='navigable floor area of the world, for context')
    parser.add_argument('--until', type=float, default=None,
                        help='truncate both runs to this simulation time')
    args = parser.parse_args()

    labels, runs = [], {}
    for spec in args.runs:
        label, _, path = spec.partition('=')
        labels.append(label)
        runs[label] = load_run(path)

    until = args.until or min(runs[k]['t'][-1] for k in labels)
    runs = {k: load_run(runs[k]['path'], until=until) for k in labels}

    os.makedirs(args.output, exist_ok=True)
    figure_policies(runs, labels, os.path.join(args.output, 'fig_policies.png'))
    figure_compare(runs, labels, os.path.join(args.output, 'fig_compare.png'),
                   reachable=args.reachable)
    for key in labels:
        if runs[key]['planner']:
            figure_planner(runs[key], os.path.join(args.output, 'fig_planner.png'))
            break

    summarise(runs, labels, reachable=args.reachable)


if __name__ == '__main__':
    main()
