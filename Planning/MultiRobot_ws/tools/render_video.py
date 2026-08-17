#!/usr/bin/env python3
"""Replay a recorded run as a video.

Consumes the same run directory as `render_run.py` — the fused-grid snapshots
and `coverage.csv` — and writes an MP4 plus a GIF showing, frame by frame:

    left    the fused fleet grid M as it accumulates, with each agent's
            estimated trajectory drawn over it and its current pose marked
    right   explored area against time, fleet and per agent, drawn up to the
            frame's own timestamp, over the link strip

The two panels share a time cursor, so the plateaux in the coverage trace can be
read against what the agents were doing when they occurred. The pose markers are
filled while an agent's link is up and hollow while it is down: a hollow marker
moving over territory that is not appearing in the fused grid is the entire
point of the connectivity model, and it is far more legible in motion than in
any single figure.

Trajectories are the SLAM estimate rather than simulator ground truth, for the
same reason the maps are: the video shows what the fleet believes, and a
trajectory that visibly disagrees with the walls it drew is information, not an
artefact to be corrected away.

    python3 tools/render_video.py <run_dir> -o docs/figures/iter2

Requires ffmpeg on PATH.
"""

import argparse
import csv
import glob
import os
import re
import shutil
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mrs_style as style  # noqa: E402
from render_run import common_canvas, composite, extent_of, read_grid  # noqa: E402

style.apply()

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.ticker import AutoMinorLocator  # noqa: E402


def read_coverage(path):
    """Parse coverage.csv into times, fused areas, and per-agent series."""
    # As in render_run.py: t = 0 rows precede the simulator's clock and carry
    # the merger's empty default canvas, not a state of the run.
    with open(path) as handle:
        rows = [r for r in csv.DictReader(handle) if float(r['sim_time']) > 0.0]
    if not rows:
        raise SystemExit(f'{path} holds no samples after the clock started')

    names = [k[:-len('_area_m2')] for k in rows[0]
             if k.endswith('_area_m2') and k != 'merged_area_m2']

    def column(key, cast=float, missing=np.nan):
        return np.array([cast(r[key]) if r.get(key) not in (None, '') else missing
                         for r in rows])

    agents = {}
    for name in names:
        agents[name] = dict(
            area=column(f'{name}_area_m2'),
            linked=column(f'{name}_linked', lambda v: float(v) > 0.5, missing=True),
            x=column(f'{name}_x'),
            y=column(f'{name}_y'),
            yaw=column(f'{name}_yaw'))

    return column('sim_time'), column('merged_area_m2'), agents


def nearest_sample(times, t):
    return int(np.argmin(np.abs(times - t)))


def render(run_dir, out_dir, stride, fps, width_px, keep_frames=False):
    fused_paths = sorted(p for p in glob.glob(os.path.join(run_dir, 'frame_*.grid'))
                         if re.fullmatch(r'frame_\d+\.grid', os.path.basename(p)))
    if not fused_paths:
        raise SystemExit(f'no fused frame_*.grid snapshots in {run_dir}')

    frames = [g for g in (read_grid(p) for p in fused_paths[::stride])
              if g[1]['t'] > 0.0]
    times, merged, agents = read_coverage(os.path.join(run_dir, 'coverage.csv'))

    # The canvas is fixed to the final extent for the whole video: a viewport
    # that tracked the current extent would hide the growth it exists to show.
    canvas = common_canvas([m for _, m in frames])
    extent = extent_of(canvas)

    fig = plt.figure(figsize=(width_px / 110.0, width_px / 110.0 * 0.46))
    fig.set_dpi(110)
    grid = fig.add_gridspec(2, 2, width_ratios=[1.05, 1.0], height_ratios=[5, 1],
                            wspace=0.16, hspace=0.1, left=0.07, right=0.985,
                            top=0.92, bottom=0.12)
    ax_map = fig.add_subplot(grid[:, 0])
    ax_cov = fig.add_subplot(grid[0, 1])
    ax_link = fig.add_subplot(grid[1, 1], sharex=ax_cov)

    image = ax_map.imshow(composite(*frames[0], canvas), cmap=style.OCC_CMAP,
                          norm=style.OCC_NORM, origin='lower', extent=extent,
                          interpolation='nearest', zorder=1)
    ax_map.set_aspect('equal')
    ax_map.set_xlabel(r'$x$ [m]')
    ax_map.set_ylabel(r'$y$ [m]')
    ax_map.xaxis.set_minor_locator(AutoMinorLocator(2))
    ax_map.yaxis.set_minor_locator(AutoMinorLocator(2))
    ax_map.set_xlim(extent[0], extent[1])
    ax_map.set_ylim(extent[2], extent[3])
    style.scale_bar(ax_map, 5.0, extent)
    title = ax_map.set_title('')

    names = sorted(agents)
    trails, markers = {}, {}
    for index, name in enumerate(names):
        colour = style.SERIES[index % len(style.SERIES)]
        trails[name], = ax_map.plot([], [], lw=1.0, color=colour, alpha=0.9, zorder=3)
        markers[name], = ax_map.plot([], [], marker='o', ms=5.0, mew=1.1,
                                     color=colour, zorder=4, linestyle='none')

    ax_cov.set_ylabel(r'explored area $\;[\mathrm{m}^2]$')
    ax_cov.set_xlim(times[0], times[-1])
    ax_cov.set_ylim(0, float(np.nanmax(merged)) * 1.08 + 1.0)
    ax_cov.yaxis.set_minor_locator(AutoMinorLocator(2))
    ax_cov.tick_params(labelbottom=False)

    fused_line, = ax_cov.plot([], [], color=style.RULE, lw=1.5, zorder=4,
                              label=r'fleet $M$')
    agent_lines = {}
    for index, name in enumerate(names):
        agent_lines[name], = ax_cov.plot(
            [], [], lw=0.95, color=style.SERIES[index % len(style.SERIES)],
            linestyle=style.SERIES_DASH[index % len(style.SERIES_DASH)], zorder=3,
            label=rf'agent {name[-1]} ($m_{{{name[-1]}}}$)')
    ax_cov.legend(loc='upper left', ncol=2)

    # Link strip: the whole run is drawn once as background and a cursor sweeps
    # it, so the viewer sees the outage that is about to happen as well as the
    # one that just did.
    for index, name in enumerate(names):
        colour = style.SERIES[index % len(style.SERIES)]
        row = len(names) - 1 - index
        ax_link.broken_barh([(times[0], times[-1] - times[0])], (row + 0.12, 0.76),
                            facecolors='#f2f2f0', edgecolors='none')
        down = ~agents[name]['linked'].astype(bool)
        start = None
        for sample, flag in enumerate(down):
            if flag and start is None:
                start = times[sample]
            elif not flag and start is not None:
                ax_link.broken_barh([(start, times[sample] - start)], (row + 0.12, 0.76),
                                    facecolors=colour, edgecolors='none', alpha=0.85)
                start = None
        if start is not None:
            ax_link.broken_barh([(start, times[-1] - start)], (row + 0.12, 0.76),
                                facecolors=colour, edgecolors='none', alpha=0.85)

    ax_link.set_ylim(0, len(names))
    ax_link.set_yticks([len(names) - 0.5 - i for i in range(len(names))])
    ax_link.set_yticklabels([rf'$\ell_{{{n[-1]}}}$' for n in names])
    ax_link.tick_params(axis='y', length=0)
    ax_link.set_xlabel(r'simulation time $t\;[\mathrm{s}]$')
    ax_link.xaxis.set_minor_locator(AutoMinorLocator(2))
    ax_link.text(1.005, 0.5, 'link down', transform=ax_link.transAxes,
                 va='center', ha='left', fontsize=6.5, color=style.RULE)

    cursors = [ax.axvline(times[0], color=style.RULE, lw=0.7, alpha=0.55, zorder=6)
               for ax in (ax_cov, ax_link)]

    work_dir = os.path.join(out_dir, '_frames')
    if os.path.isdir(work_dir):
        shutil.rmtree(work_dir)
    os.makedirs(work_dir)

    for number, (cells, meta) in enumerate(frames):
        sample = nearest_sample(times, meta['t'])
        image.set_data(composite(cells, meta, canvas))
        title.set_text(rf'fused fleet grid $M$,  $t = {meta["t"]:.0f}$ s')

        for name in names:
            state = agents[name]
            xs, ys = state['x'][:sample + 1], state['y'][:sample + 1]
            valid = np.isfinite(xs) & np.isfinite(ys)
            trails[name].set_data(xs[valid], ys[valid])

            if valid.any() and np.isfinite(state['x'][sample]):
                markers[name].set_data([state['x'][sample]], [state['y'][sample]])
                # Hollow while the link is down: the agent goes on mapping, but
                # nothing it maps is reaching the fleet.
                linked = bool(state['linked'][sample])
                markers[name].set_markerfacecolor(
                    markers[name].get_color() if linked else 'white')
            else:
                markers[name].set_data([], [])

        fused_line.set_data(times[:sample + 1], merged[:sample + 1])
        for name in names:
            agent_lines[name].set_data(times[:sample + 1], agents[name]['area'][:sample + 1])
        for cursor in cursors:
            cursor.set_xdata([times[sample], times[sample]])

        # Fixed dpi and no tight bounding box: a video needs every frame to be
        # exactly the same size, and `bbox_inches='tight'` sizes each frame to
        # its own content.
        fig.savefig(os.path.join(work_dir, f'{number:05d}.png'), dpi=110,
                    bbox_inches=None, pad_inches=0.0)

    plt.close(fig)
    print(f'rendered {len(frames)} frames')

    mp4 = os.path.join(out_dir, 'fig_run.mp4')
    gif = os.path.join(out_dir, 'fig_run.gif')
    encode(work_dir, mp4, gif, fps)

    if not keep_frames:
        shutil.rmtree(work_dir)
    return mp4, gif


def encode(frame_dir, mp4, gif, fps):
    pattern = os.path.join(frame_dir, '%05d.png')

    # yuv420p and the even-dimension pad keep the MP4 playable in browsers and
    # in the players that refuse odd frame sizes.
    run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(fps), '-i', pattern,
         '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2', '-c:v', 'libx264', '-crf', '20',
         '-pix_fmt', 'yuv420p', mp4])

    # A GIF is palettised, so it needs its own palette pass: the default 216
    # colour web palette turns the greyscale occupancy ramp into visible bands.
    palette = os.path.join(frame_dir, 'palette.png')
    scale = 'scale=900:-1:flags=lanczos'
    run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(fps), '-i', pattern,
         '-vf', f'{scale},palettegen=max_colors=128:stats_mode=diff', palette])
    run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(fps), '-i', pattern,
         '-i', palette, '-lavfi', f'{scale}[v];[v][1:v]paletteuse=dither=bayer:bayer_scale=3',
         '-loop', '0', gif])

    for path in (mp4, gif):
        print(f'wrote {path} ({os.path.getsize(path) / 1e6:.1f} MB)')


def run(command):
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0:
        raise SystemExit(f'{command[0]} failed:\n{result.stderr}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir')
    parser.add_argument('-o', '--out-dir', default=None)
    parser.add_argument('--stride', type=int, default=1,
                        help='keep every Nth snapshot')
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--width', type=int, default=1400,
                        help='rendered frame width in pixels')
    parser.add_argument('--keep-frames', action='store_true')
    args = parser.parse_args()

    out_dir = args.out_dir or os.path.join(args.run_dir, 'figures')
    os.makedirs(out_dir, exist_ok=True)
    render(args.run_dir, out_dir, args.stride, args.fps, args.width, args.keep_frames)


if __name__ == '__main__':
    main()
