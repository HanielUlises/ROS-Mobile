#!/usr/bin/env python3
"""Render the figures for a recorded multi-robot SLAM run.

Consumes the output of `map_recorder_node` (a directory of `frame_*.grid`
snapshots plus `coverage.csv`) and produces:

    final_map.png    the merged occupancy grid at the end of the run
    map_growth.gif   the merged grid evolving over the run
    coverage.png     explored area against time, with link outages shaded

All frames are composited onto a single canvas covering the union of every
snapshot's extent, so the animation shows the map growing rather than the
viewport jumping.

    python3 tools/render_run.py <run_dir> [-o <figure_dir>]
"""

import argparse
import csv
import glob
import os
import struct

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import ListedColormap, BoundaryNorm  # noqa: E402

UNKNOWN, FREE, OCCUPIED = 0, 1, 2

# Paper-like greyscale: unknown reads as absent, free as page, occupied as ink.
CMAP = ListedColormap(['#d9d9d9', '#ffffff', '#1a1a1a'])
NORM = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], CMAP.N)

ACCENT = ['#0b6fa4', '#c2610b', '#5c7a1e']


def read_grid(path):
    """Parse one `.grid` snapshot into (cells, meta)."""
    with open(path, 'rb') as handle:
        header = b''
        while not header.endswith(b'\n'):
            header += handle.read(1)
        fields = header.decode().split()
        if fields[0] != 'MRSGRID':
            raise ValueError(f'{path}: not a MRSGRID snapshot')
        width, height = int(fields[1]), int(fields[2])
        res, ox, oy, stamp = (float(f) for f in fields[3:7])
        raw = handle.read(width * height)

    cells = np.array(struct.unpack(f'{len(raw)}b', raw), dtype=np.int8)
    cells = cells.reshape(height, width)
    return cells, dict(width=width, height=height, res=res, ox=ox, oy=oy, t=stamp)


def classify(cells, occupied_threshold=50):
    """Map raw occupancy values onto the three display classes."""
    out = np.full(cells.shape, UNKNOWN, dtype=np.uint8)
    known = cells >= 0
    out[known & (cells < occupied_threshold)] = FREE
    out[known & (cells >= occupied_threshold)] = OCCUPIED
    return out


def common_canvas(metas):
    """Extent covering every snapshot, as (min_x, min_y, width, height, res)."""
    res = metas[0]['res']
    min_x = min(m['ox'] for m in metas)
    min_y = min(m['oy'] for m in metas)
    max_x = max(m['ox'] + m['width'] * m['res'] for m in metas)
    max_y = max(m['oy'] + m['height'] * m['res'] for m in metas)
    width = int(round((max_x - min_x) / res))
    height = int(round((max_y - min_y) / res))
    return min_x, min_y, width, height, res


def composite(cells, meta, canvas):
    """Place one snapshot onto the shared canvas."""
    min_x, min_y, width, height, res = canvas
    out = np.full((height, width), UNKNOWN, dtype=np.uint8)
    x0 = int(round((meta['ox'] - min_x) / res))
    y0 = int(round((meta['oy'] - min_y) / res))
    classified = classify(cells)
    h, w = classified.shape
    x1, y1 = min(x0 + w, width), min(y0 + h, height)
    if x1 > x0 and y1 > y0:
        out[y0:y1, x0:x1] = classified[:y1 - y0, :x1 - x0]
    return out


def frame_extent(canvas):
    min_x, min_y, width, height, res = canvas
    return [min_x, min_x + width * res, min_y, min_y + height * res]


def style_axes(ax, canvas, title):
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(title, fontsize=11, pad=8)
    ax.set_aspect('equal')
    for spine in ax.spines.values():
        spine.set_linewidth(0.6)
        spine.set_color('#444444')
    ax.tick_params(labelsize=8, width=0.6, color='#444444')
    del canvas


def render_final_map(frames, canvas, out_path):
    cells, meta = frames[-1]
    image = composite(cells, meta, canvas)

    fig, ax = plt.subplots(figsize=(7.2, 5.0), dpi=200)
    ax.imshow(image, cmap=CMAP, norm=NORM, origin='lower',
              extent=frame_extent(canvas), interpolation='nearest')
    style_axes(ax, canvas, f'Merged occupancy grid, t = {meta["t"]:.0f} s')

    handles = [
        plt.Rectangle((0, 0), 1, 1, facecolor='#ffffff', edgecolor='#444444', lw=0.6),
        plt.Rectangle((0, 0), 1, 1, facecolor='#1a1a1a', edgecolor='#444444', lw=0.6),
        plt.Rectangle((0, 0), 1, 1, facecolor='#d9d9d9', edgecolor='#444444', lw=0.6),
    ]
    ax.legend(handles, ['free', 'occupied', 'unknown'],
              loc='lower right', fontsize=8, framealpha=0.95)

    fig.tight_layout()
    fig.savefig(out_path, bbox_inches='tight')
    plt.close(fig)
    print(f'wrote {out_path}')


def render_growth_gif(frames, canvas, out_path, fps=8):
    from matplotlib.animation import FuncAnimation, PillowWriter

    fig, ax = plt.subplots(figsize=(6.4, 4.4), dpi=130)
    image = ax.imshow(composite(*frames[0], canvas), cmap=CMAP, norm=NORM,
                      origin='lower', extent=frame_extent(canvas),
                      interpolation='nearest')
    style_axes(ax, canvas, '')
    caption = ax.set_title('t = 0 s', fontsize=11, pad=8)
    fig.tight_layout()

    def update(index):
        cells, meta = frames[index]
        image.set_data(composite(cells, meta, canvas))
        caption.set_text(f'Merged occupancy grid,  t = {meta["t"]:.0f} s')
        return image, caption

    # Hold on the final state so the finished map is readable before the loop.
    order = list(range(len(frames))) + [len(frames) - 1] * (2 * fps)
    animation = FuncAnimation(fig, update, frames=order, blit=False)
    animation.save(out_path, writer=PillowWriter(fps=fps))
    plt.close(fig)
    print(f'wrote {out_path}')


def outage_spans(times, linked):
    """Contiguous [start, end] intervals where a link is down."""
    spans, start = [], None
    for t, up in zip(times, linked):
        if not up and start is None:
            start = t
        elif up and start is not None:
            spans.append((start, t))
            start = None
    if start is not None:
        spans.append((start, times[-1]))
    return spans


def render_coverage(csv_path, out_path):
    with open(csv_path) as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        print('coverage.csv is empty, skipping coverage figure')
        return

    robots = [key[:-len('_area_m2')] for key in rows[0]
              if key.endswith('_area_m2') and key != 'merged_area_m2']

    times = np.array([float(r['sim_time']) for r in rows])
    merged = np.array([float(r['merged_area_m2']) for r in rows])

    fig, ax = plt.subplots(figsize=(7.2, 4.0), dpi=200)

    # Shade outages first so the curves stay on top.
    labelled = False
    for index, name in enumerate(robots):
        linked = [r[f'{name}_linked'] == '1' for r in rows]
        for start, end in outage_spans(times, linked):
            ax.axvspan(start, end, color=ACCENT[index % len(ACCENT)], alpha=0.10,
                       lw=0, label=None if labelled else 'link down')
            labelled = True

    for index, name in enumerate(robots):
        area = np.array([float(r[f'{name}_area_m2']) for r in rows])
        ax.plot(times, area, lw=1.4, color=ACCENT[index % len(ACCENT)],
                label=f'{name} (own map)')

    ax.plot(times, merged, lw=2.2, color='#1a1a1a', label='fleet (merged map)')

    ax.set_xlabel('simulation time [s]')
    ax.set_ylabel(r'explored area [m$^2$]')
    ax.set_title('Explored area under intermittent connectivity', fontsize=11, pad=8)
    ax.grid(True, lw=0.4, color='#cccccc', alpha=0.8)
    ax.set_axisbelow(True)
    for spine in ax.spines.values():
        spine.set_linewidth(0.6)
        spine.set_color('#444444')
    ax.tick_params(labelsize=8, width=0.6, color='#444444')
    ax.legend(fontsize=8, loc='lower right', framealpha=0.95)

    fig.tight_layout()
    fig.savefig(out_path, bbox_inches='tight')
    plt.close(fig)
    print(f'wrote {out_path}')

    print(f'  final merged area   : {merged[-1]:.1f} m^2')
    for name in robots:
        own = float(rows[-1][f'{name}_area_m2'])
        print(f'  final {name:<12}: {own:.1f} m^2 ({100 * own / merged[-1]:.0f}% of merged)')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir')
    parser.add_argument('-o', '--out-dir', default=None)
    parser.add_argument('--gif-stride', type=int, default=1,
                        help='keep every Nth snapshot in the animation')
    args = parser.parse_args()

    out_dir = args.out_dir or os.path.join(args.run_dir, 'figures')
    os.makedirs(out_dir, exist_ok=True)

    paths = sorted(glob.glob(os.path.join(args.run_dir, 'frame_*.grid')))
    if not paths:
        raise SystemExit(f'no frame_*.grid snapshots in {args.run_dir}')

    frames = [read_grid(p) for p in paths]
    canvas = common_canvas([meta for _, meta in frames])
    print(f'{len(frames)} snapshots, canvas {canvas[2]}x{canvas[3]} cells '
          f'@ {canvas[4]:.3f} m')

    render_final_map(frames, canvas, os.path.join(out_dir, 'final_map.png'))
    render_growth_gif(frames[::args.gif_stride], canvas,
                      os.path.join(out_dir, 'map_growth.gif'))
    render_coverage(os.path.join(args.run_dir, 'coverage.csv'),
                    os.path.join(out_dir, 'coverage.png'))


if __name__ == '__main__':
    main()
