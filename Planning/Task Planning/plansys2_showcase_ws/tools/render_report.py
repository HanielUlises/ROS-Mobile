#!/usr/bin/env python3
"""Render the figures for the warehouse showcase reports.

Reporting only: every number this draws was produced by the C++ tools and nodes
in `src/`, and nothing here recomputes any of it. The division is deliberate —
plots are the one place where a scripting language earns its keep, and keeping
them out of the workspace's own packages keeps the workspace buildable without
a plotting stack.

    python3 tools/render_report.py --runs /tmp/warehouse_showcase \
        --scaling docs/data/planner_scaling.csv -o docs/figures
"""

import argparse
import csv
import os
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt   # noqa: E402
import numpy as np                # noqa: E402
import yaml                       # noqa: E402

INK = '#1a1a1a'
SERIES = ['#2f6fb0', '#c0392b', '#1f7a3f', '#b8860b', '#7d3c98', '#0e7490']


def apply_style():
    plt.rcParams.update({
        'figure.dpi': 200,
        'savefig.dpi': 200,
        'font.size': 8,
        'axes.titlesize': 8.5,
        'axes.labelsize': 8,
        'axes.edgecolor': INK,
        'axes.linewidth': 0.7,
        'axes.spines.top': False,
        'axes.spines.right': False,
        'xtick.labelsize': 7,
        'ytick.labelsize': 7,
        'legend.fontsize': 7,
        'legend.frameon': False,
        'grid.color': '#d8d8d8',
        'grid.linewidth': 0.5,
        'text.usetex': False,
        'mathtext.fontset': 'dejavuserif',
        'font.family': 'DejaVu Serif',
    })


def read_csv(path):
    with open(path) as handle:
        return list(csv.DictReader(handle))


# --------------------------------------------------------------------------
# the roadmap over the map it was derived from
# --------------------------------------------------------------------------

def figure_roadmap(roadmap_path, map_yaml, out_path):
    from PIL import Image

    with open(roadmap_path) as handle:
        roadmap = yaml.safe_load(handle)
    with open(map_yaml) as handle:
        meta = yaml.safe_load(handle)
    image = np.array(Image.open(
        os.path.join(os.path.dirname(map_yaml), meta['image'])).convert('L'))

    res = float(meta['resolution'])
    ox, oy = meta['origin'][0], meta['origin'][1]
    height, width = image.shape
    extent = [ox, ox + width * res, oy, oy + height * res]

    fig, ax = plt.subplots(figsize=(4.2, 5.9))
    ax.imshow(image, cmap='gray', origin='upper', extent=extent,
              interpolation='nearest', vmin=0, vmax=255)

    positions = {n: (v['x'], v['y']) for n, v in roadmap['waypoints'].items()}
    for edge in roadmap['edges']:
        a, b = positions[edge['from']], positions[edge['to']]
        ax.plot([a[0], b[0]], [a[1], b[1]], lw=0.7, color=SERIES[0], zorder=3, alpha=0.8)

    style = {'dock': (SERIES[1], 's', 5.0), 'storage': (SERIES[2], 'o', 4.5),
             'charger': (SERIES[3], 'D', 4.0), 'corridor': (SERIES[0], '.', 4.0)}
    for name, entry in roadmap['waypoints'].items():
        colour, marker, size = style[entry['kind']]
        ax.plot([entry['x']], [entry['y']], marker=marker, ms=size, color=colour,
                linestyle='none', zorder=4)
        if entry['kind'] != 'corridor':
            ax.annotate(name, (entry['x'], entry['y']), textcoords='offset points',
                        xytext=(5, 2.5), fontsize=5.5, color=colour, zorder=5)

    ax.set_xlabel(r'$x$ [m]')
    ax.set_ylabel(r'$y$ [m]')
    ax.set_aspect('equal')
    ax.set_title(f"{len(positions)} waypoints, {len(roadmap['edges'])} edges")
    fig.savefig(out_path, bbox_inches='tight')
    plt.close(fig)
    print(f'wrote {out_path}')


# --------------------------------------------------------------------------
# one run: the plan as a schedule, and execution against it
# --------------------------------------------------------------------------

def figure_run(run_dir, out_path, title):
    plan = read_csv(os.path.join(run_dir, 'plan.csv'))
    progress = read_csv(os.path.join(run_dir, 'execution.csv'))

    def robot_of(action):
        return action.strip('()').split()[1]

    def verb_of(action):
        return action.strip('()').split()[0]

    robots = sorted({robot_of(row['action']) for row in plan})
    colour = {'move': SERIES[0], 'enter_dock': SERIES[5], 'leave_dock': SERIES[5],
              'pick': SERIES[2], 'drop': SERIES[1]}

    fig, axes = plt.subplots(2, 1, figsize=(6.6, 3.4), sharex=True,
                             gridspec_kw=dict(height_ratios=[2.0, 1.0], hspace=0.18))

    ax = axes[0]
    for row in plan:
        robot = robot_of(row['action'])
        verb = verb_of(row['action'])
        start = float(row['start_s'])
        duration = float(row['duration_s'])
        y = robots.index(robot)
        ax.barh(y, duration, left=start, height=0.55,
                color=colour.get(verb, '#888888'), edgecolor='white', linewidth=0.4)
    ax.set_yticks(range(len(robots)))
    ax.set_yticklabels(robots)
    ax.set_ylabel('robot')
    ax.set_title(title)
    ax.invert_yaxis()
    handles = [plt.Rectangle((0, 0), 1, 1, color=colour[k])
               for k in ('move', 'pick', 'drop', 'enter_dock')]
    ax.legend(handles, ['move', 'pick', 'drop', 'enter / leave dock'],
              loc='upper right', ncol=4)

    ax = axes[1]
    t = np.array([float(r['wall_time_s']) for r in progress])
    done = np.array([float(r['completed_actions']) for r in progress])
    total = float(progress[0]['total_actions']) if progress else 1.0
    ax.plot(t, done, lw=1.2, color=INK, label='actions completed')
    ax.axhline(total, lw=0.7, ls=':', color='#888888')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('actions')
    ax.legend(loc='lower right')
    ax.grid(True, axis='y')

    fig.savefig(out_path, bbox_inches='tight')
    plt.close(fig)
    print(f'wrote {out_path}')


# --------------------------------------------------------------------------
# the planner as the fleet and the workload grow
# --------------------------------------------------------------------------

def figure_scaling(scaling_csv, out_path):
    rows = read_csv(scaling_csv)
    by_crates = defaultdict(list)
    for row in rows:
        by_crates[int(row['crates'])].append(row)

    fig, axes = plt.subplots(1, 3, figsize=(7.4, 2.5),
                             gridspec_kw=dict(wspace=0.32))

    ax = axes[0]
    for index, crates in enumerate(sorted(by_crates)):
        subset = sorted(by_crates[crates], key=lambda r: int(r['robots']))
        solved = [r for r in subset if r['solved'] == '1']
        ax.plot([int(r['robots']) for r in solved],
                [float(r['makespan_s']) for r in solved],
                marker='o', ms=3, lw=1.1, color=SERIES[index % len(SERIES)],
                label=f'{crates} crates')
    ax.set_xlabel('robots')
    ax.set_ylabel('makespan [s]')
    ax.grid(True)
    ax.legend(ncol=2)

    ax = axes[1]
    for index, crates in enumerate(sorted(by_crates)):
        subset = sorted(by_crates[crates], key=lambda r: int(r['robots']))
        solved = [r for r in subset if r['solved'] == '1']
        ax.plot([int(r['robots']) for r in solved],
                [float(r['planning_seconds']) for r in solved],
                marker='o', ms=3, lw=1.1, color=SERIES[index % len(SERIES)])
        for r in subset:
            if r['timed_out'] == '1':
                ax.plot([int(r['robots'])], [float(r['planning_seconds'])],
                        marker='x', ms=6, color=SERIES[index % len(SERIES)])
    ax.set_yscale('log')
    ax.set_xlabel('robots')
    ax.set_ylabel('planning time [s]')
    ax.set_title('x = timeout')
    ax.grid(True, which='both')

    ax = axes[2]
    for index, crates in enumerate(sorted(by_crates)):
        subset = sorted(by_crates[crates], key=lambda r: int(r['robots']))
        solved = [r for r in subset if r['solved'] == '1']
        ax.plot([int(r['robots']) for r in solved],
                [float(r['peak_concurrency']) for r in solved],
                marker='o', ms=3, lw=1.1, color=SERIES[index % len(SERIES)])
    limit = max(int(r['robots']) for r in rows)
    ax.plot([1, limit], [1, limit], lw=0.7, ls='--', color='#888888')
    ax.set_xlabel('robots')
    ax.set_ylabel('peak concurrent actions')
    ax.grid(True)

    fig.savefig(out_path, bbox_inches='tight')
    plt.close(fig)
    print(f'wrote {out_path}')


# --------------------------------------------------------------------------
# what the fleet actually did, run by run
# --------------------------------------------------------------------------

def figure_fleet(run_dirs, out_path):
    sizes, makespans, executions, distances, idle_fraction = [], [], [], [], []
    for run_dir in run_dirs:
        summary = read_csv(os.path.join(run_dir, 'summary.csv'))[0]
        fleet = read_csv(os.path.join(run_dir, 'fleet_summary.csv'))
        plan = read_csv(os.path.join(run_dir, 'plan.csv'))
        makespan = max(float(r['start_s']) + float(r['duration_s']) for r in plan)

        sizes.append(int(summary['robots']))
        makespans.append(makespan)
        executions.append(float(summary['execution_seconds']))
        distances.append(sum(float(r['distance_m']) for r in fleet))
        moving = sum(float(r['moving_s']) for r in fleet)
        idling = sum(float(r['idle_s']) for r in fleet)
        idle_fraction.append(100.0 * idling / max(moving + idling, 1e-9))

    order = np.argsort(sizes)
    sizes = np.array(sizes)[order]
    makespans = np.array(makespans)[order]
    executions = np.array(executions)[order]
    distances = np.array(distances)[order]
    idle_fraction = np.array(idle_fraction)[order]

    fig, axes = plt.subplots(1, 3, figsize=(7.4, 2.4),
                             gridspec_kw=dict(wspace=0.34))

    ax = axes[0]
    width = 0.36
    ax.bar(sizes - width / 2, makespans, width, color=SERIES[0], label='planned makespan')
    ax.bar(sizes + width / 2, executions, width, color=SERIES[1], label='measured')
    ax.set_xlabel('robots')
    ax.set_ylabel('time [s]')
    ax.set_xticks(sizes)
    ax.legend()
    ax.grid(True, axis='y')

    ax = axes[1]
    ax.bar(sizes, distances, 0.5, color=SERIES[2])
    ax.set_xlabel('robots')
    ax.set_ylabel('fleet distance [m]')
    ax.set_xticks(sizes)
    ax.grid(True, axis='y')

    ax = axes[2]
    ax.bar(sizes, idle_fraction, 0.5, color=SERIES[3])
    ax.set_xlabel('robots')
    ax.set_ylabel('time held still [%]')
    ax.set_xticks(sizes)
    ax.grid(True, axis='y')

    fig.savefig(out_path, bbox_inches='tight')
    plt.close(fig)
    print(f'wrote {out_path}')


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--roadmap', default=os.path.join(
        here, 'src/warehouse_planning/config/roadmap.yaml'))
    parser.add_argument('--map', default=os.path.join(
        here, 'src/warehouse_planning/maps/warehouse.yaml'))
    parser.add_argument('--scaling', default=os.path.join(
        here, 'docs/data/planner_scaling.csv'))
    parser.add_argument('--runs', nargs='*', default=[],
                        help='run directories, one per fleet size')
    parser.add_argument('-o', '--output', default=os.path.join(here, 'docs/figures'))
    args = parser.parse_args()

    apply_style()
    os.makedirs(args.output, exist_ok=True)

    figure_roadmap(args.roadmap, args.map, os.path.join(args.output, 'fig_roadmap.png'))
    if os.path.exists(args.scaling):
        figure_scaling(args.scaling, os.path.join(args.output, 'fig_scaling.png'))
    if args.runs:
        single = [r for r in args.runs
                  if read_csv(os.path.join(r, 'summary.csv'))[0]['robots'] == '1']
        if single:
            figure_run(single[0], os.path.join(args.output, 'fig_single_run.png'),
                       'single agent: three crates, 26 actions')
        largest = max(args.runs,
                      key=lambda r: int(read_csv(os.path.join(r, 'summary.csv'))[0]['robots']))
        figure_run(largest, os.path.join(args.output, 'fig_fleet_run.png'),
                   'four agents: the same three crates')
        figure_fleet(args.runs, os.path.join(args.output, 'fig_fleet.png'))


if __name__ == '__main__':
    main()
