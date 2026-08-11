#!/usr/bin/env python3
"""Render the figures for a recorded multi-robot SLAM run.

Consumes the output of `map_recorder_node` — `frame_*.grid` snapshots of the
fused grid, `frame_*_<robot>.grid` snapshots of each agent's own grid, and
`coverage.csv` — and produces:

    fig_maps.png      per-agent estimates beside the fused estimate
    fig_coverage.png  explored area against time, with link outages marked
    fig_growth.gif    the fused grid evolving over the run

Every snapshot carries the SE(2) transform of its own frame into the global
frame, so per-agent and fused grids are composited onto one common canvas and
are directly comparable panel to panel.

    python3 tools/render_run.py <run_dir> [-o <figure_dir>]
"""

import argparse
import csv
import glob
import os
import re
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mrs_style as style  # noqa: E402

style.apply()

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.ticker import AutoMinorLocator  # noqa: E402


# --------------------------------------------------------------------------
# snapshot IO
# --------------------------------------------------------------------------

def read_grid(path):
    """Parse one `.grid` snapshot into (cells, meta)."""
    with open(path, 'rb') as handle:
        header = b''
        while not header.endswith(b'\n'):
            byte = handle.read(1)
            if not byte:
                raise ValueError(f'{path}: truncated header')
            header += byte
        fields = header.decode().split()
        if fields[0] != 'MRSGRID':
            raise ValueError(f'{path}: not a MRSGRID snapshot')
        width, height = int(fields[1]), int(fields[2])
        res, ox, oy, stamp = (float(f) for f in fields[3:7])
        # Older snapshots carry no pose; treat them as already global.
        tx, ty, tyaw = (float(f) for f in fields[7:10]) if len(fields) >= 10 else (0.0, 0.0, 0.0)
        raw = handle.read(width * height)

    cells = np.frombuffer(raw, dtype=np.int8).reshape(height, width)
    return cells, dict(width=width, height=height, res=res, ox=ox, oy=oy,
                       t=stamp, tx=tx, ty=ty, tyaw=tyaw)


def classify(cells, occupied_threshold=50):
    out = np.full(cells.shape, style.UNKNOWN, dtype=np.uint8)
    known = cells >= 0
    out[known & (cells < occupied_threshold)] = style.FREE
    out[known & (cells >= occupied_threshold)] = style.OCCUPIED
    return out


def grid_corners(meta):
    """The snapshot's four corners in global metres."""
    w = meta['width'] * meta['res']
    h = meta['height'] * meta['res']
    local = np.array([[meta['ox'], meta['oy']],
                      [meta['ox'] + w, meta['oy']],
                      [meta['ox'], meta['oy'] + h],
                      [meta['ox'] + w, meta['oy'] + h]])
    c, s = np.cos(meta['tyaw']), np.sin(meta['tyaw'])
    rot = np.array([[c, -s], [s, c]])
    return local @ rot.T + np.array([meta['tx'], meta['ty']])


def common_canvas(metas, res=None):
    res = res or metas[0]['res']
    corners = np.vstack([grid_corners(m) for m in metas])
    min_x, min_y = corners.min(axis=0)
    max_x, max_y = corners.max(axis=0)
    width = int(np.ceil((max_x - min_x) / res)) + 1
    height = int(np.ceil((max_y - min_y) / res)) + 1
    return min_x, min_y, width, height, res


def composite(cells, meta, canvas):
    """Place one snapshot onto the shared canvas, applying its SE(2) pose."""
    min_x, min_y, width, height, res = canvas
    out = np.full((height, width), style.UNKNOWN, dtype=np.uint8)
    classified = classify(cells)

    gy, gx = np.nonzero(classified != style.UNKNOWN)
    if gx.size == 0:
        return out

    lx = meta['ox'] + (gx + 0.5) * meta['res']
    ly = meta['oy'] + (gy + 0.5) * meta['res']
    c, s = np.cos(meta['tyaw']), np.sin(meta['tyaw'])
    wx = meta['tx'] + c * lx - s * ly
    wy = meta['ty'] + s * lx + c * ly

    mx = ((wx - min_x) / res).astype(np.int64)
    my = ((wy - min_y) / res).astype(np.int64)
    keep = (mx >= 0) & (my >= 0) & (mx < width) & (my < height)
    out[my[keep], mx[keep]] = classified[gy[keep], gx[keep]]
    return out


def extent_of(canvas):
    min_x, min_y, width, height, res = canvas
    return [min_x, min_x + width * res, min_y, min_y + height * res]


def dress_map_axes(ax, extent, show_y=True):
    ax.set_aspect('equal')
    ax.set_xlabel(r'$x$ [m]')
    if show_y:
        ax.set_ylabel(r'$y$ [m]')
    else:
        ax.set_yticklabels([])
    ax.xaxis.set_minor_locator(AutoMinorLocator(2))
    ax.yaxis.set_minor_locator(AutoMinorLocator(2))
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])


# --------------------------------------------------------------------------
# figures
# --------------------------------------------------------------------------

def figure_maps(final_fused, final_agents, canvas, out_path):
    """Per-agent estimates beside the fused estimate, on one canvas."""
    n = len(final_agents) + 1
    extent = extent_of(canvas)
    aspect = (extent[3] - extent[2]) / (extent[1] - extent[0])

    width = 6.9
    height = width / n * aspect + 0.85
    fig, axes = plt.subplots(1, n, figsize=(width, height),
                             gridspec_kw=dict(wspace=0.06))

    panels = [(name, cells, meta) for name, (cells, meta) in sorted(final_agents.items())]
    panels.append(('fused', *final_fused))

    tags = '(a) (b) (c) (d) (e)'.split()
    for index, (ax, (name, cells, meta)) in enumerate(zip(axes, panels)):
        ax.imshow(composite(cells, meta, canvas), cmap=style.OCC_CMAP,
                  norm=style.OCC_NORM, origin='lower', extent=extent,
                  interpolation='nearest', rasterized=True)
        dress_map_axes(ax, extent, show_y=(index == 0))

        label = r'fleet, $M$' if name == 'fused' else rf'agent {name[-1]}, $m_{{{name[-1]}}}$'
        ax.set_title(label)
        style.panel_label(ax, tags[index])

    style.scale_bar(axes[-1], 2.0, extent)

    handles = [
        plt.Rectangle((0, 0), 1, 1, facecolor='#ffffff', edgecolor=style.RULE, lw=0.6),
        plt.Rectangle((0, 0), 1, 1, facecolor='#111111', edgecolor=style.RULE, lw=0.6),
        plt.Rectangle((0, 0), 1, 1, facecolor='#e8e8e6', edgecolor=style.RULE, lw=0.6),
    ]
    fig.legend(handles, ['free', 'occupied', r'unobserved, $\mathsf{u}$'],
               loc='lower center', ncol=3, bbox_to_anchor=(0.5, -0.015),
               handlelength=1.1, handleheight=1.1, columnspacing=1.8)

    fig.savefig(out_path)
    plt.close(fig)
    print(f'wrote {out_path}')


def outage_spans(times, linked):
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


def figure_coverage(csv_path, out_path):
    with open(csv_path) as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        print('coverage.csv is empty, skipping coverage figure')
        return None

    robots = [k[:-len('_area_m2')] for k in rows[0]
              if k.endswith('_area_m2') and k != 'merged_area_m2']
    times = np.array([float(r['sim_time']) for r in rows])
    merged = np.array([float(r['merged_area_m2']) for r in rows])
    areas = {n: np.array([float(r[f'{n}_area_m2']) for r in rows]) for n in robots}

    fig, (ax, axl) = plt.subplots(
        2, 1, figsize=(6.9, 3.5), sharex=True,
        gridspec_kw=dict(height_ratios=[5, 1], hspace=0.09))

    ax.plot(times, merged, color=style.RULE, lw=1.5, zorder=4,
            label=r'fleet $M$ (fused)')
    for index, name in enumerate(robots):
        ax.plot(times, areas[name], lw=0.95,
                color=style.SERIES[index % len(style.SERIES)],
                linestyle=style.SERIES_DASH[index % len(style.SERIES_DASH)],
                zorder=3, label=rf'agent {name[-1]} ($m_{{{name[-1]}}}$, own)')

    ax.set_ylabel(r'explored area $\;[\mathrm{m}^2]$')
    ax.yaxis.set_minor_locator(AutoMinorLocator(2))
    ax.legend(loc='lower right', ncol=1)
    ax.margins(x=0.01)

    # Link state as a dedicated raster strip: outages are a discrete process and
    # deserve their own axis rather than shading smeared behind the curves.
    for index, name in enumerate(robots):
        colour = style.SERIES[index % len(style.SERIES)]
        y = len(robots) - 1 - index
        axl.broken_barh([(times[0], times[-1] - times[0])], (y + 0.12, 0.76),
                        facecolors='#f2f2f0', edgecolors='none')
        for start, end in outage_spans(times, [r[f'{name}_linked'] == '1' for r in rows]):
            axl.broken_barh([(start, end - start)], (y + 0.12, 0.76),
                            facecolors=colour, edgecolors='none', alpha=0.85)

    axl.set_ylim(0, len(robots))
    axl.set_yticks([len(robots) - 0.5 - i for i in range(len(robots))])
    axl.set_yticklabels([rf'$\ell_{{{n[-1]}}}$' for n in robots])
    axl.tick_params(axis='y', length=0)
    axl.set_xlabel(r'simulation time $t\;[\mathrm{s}]$')
    axl.xaxis.set_minor_locator(AutoMinorLocator(2))
    axl.margins(x=0.01)
    axl.text(1.005, 0.5, 'link down', transform=axl.transAxes,
             va='center', ha='left', fontsize=7.0, color=style.RULE)

    fig.savefig(out_path)
    plt.close(fig)
    print(f'wrote {out_path}')

    summary = {'merged': merged[-1]}
    summary.update({n: areas[n][-1] for n in robots})
    return summary


def figure_growth(frames, canvas, out_path, fps=8):
    from matplotlib.animation import FuncAnimation, PillowWriter

    extent = extent_of(canvas)
    aspect = (extent[3] - extent[2]) / (extent[1] - extent[0])
    fig, ax = plt.subplots(figsize=(4.2, 4.2 * aspect + 0.55))
    fig.set_dpi(150)

    image = ax.imshow(composite(*frames[0], canvas), cmap=style.OCC_CMAP,
                      norm=style.OCC_NORM, origin='lower', extent=extent,
                      interpolation='nearest')
    dress_map_axes(ax, extent)
    style.scale_bar(ax, 2.0, extent)
    caption = ax.set_title('')
    fig.tight_layout()

    def update(index):
        cells, meta = frames[index]
        image.set_data(composite(cells, meta, canvas))
        caption.set_text(rf'fused grid $M$,  $t = {meta["t"]:.0f}$ s')
        return image, caption

    order = list(range(len(frames))) + [len(frames) - 1] * (2 * fps)
    animation = FuncAnimation(fig, update, frames=order, blit=False)
    animation.save(out_path, writer=PillowWriter(fps=fps), dpi=150)
    plt.close(fig)
    print(f'wrote {out_path}')


# --------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir')
    parser.add_argument('-o', '--out-dir', default=None)
    parser.add_argument('--gif-stride', type=int, default=1,
                        help='keep every Nth snapshot in the animation')
    args = parser.parse_args()

    out_dir = args.out_dir or os.path.join(args.run_dir, 'figures')
    os.makedirs(out_dir, exist_ok=True)

    fused_paths = sorted(p for p in glob.glob(os.path.join(args.run_dir, 'frame_*.grid'))
                         if re.fullmatch(r'frame_\d+\.grid', os.path.basename(p)))
    if not fused_paths:
        raise SystemExit(f'no fused frame_*.grid snapshots in {args.run_dir}')

    fused = [read_grid(p) for p in fused_paths]

    agent_paths = {}
    for path in glob.glob(os.path.join(args.run_dir, 'frame_*_*.grid')):
        match = re.fullmatch(r'frame_(\d+)_(.+)\.grid', os.path.basename(path))
        if match:
            agent_paths.setdefault(match.group(2), []).append((int(match.group(1)), path))
    final_agents = {name: read_grid(max(items)[1]) for name, items in agent_paths.items()}

    metas = [m for _, m in fused] + [m for _, m in final_agents.values()]
    canvas = common_canvas(metas)
    print(f'{len(fused)} fused snapshots, {len(final_agents)} agent grids, '
          f'canvas {canvas[2]}x{canvas[3]} cells @ {canvas[4]:.3f} m')

    if final_agents:
        figure_maps(fused[-1], final_agents, canvas,
                    os.path.join(out_dir, 'fig_maps.png'))

    summary = figure_coverage(os.path.join(args.run_dir, 'coverage.csv'),
                              os.path.join(out_dir, 'fig_coverage.png'))
    figure_growth(fused[::args.gif_stride], canvas,
                  os.path.join(out_dir, 'fig_growth.gif'))

    if summary:
        print(f'  fused           : {summary.pop("merged"):.1f} m^2')
        for name, value in sorted(summary.items()):
            print(f'  {name:<16}: {value:.1f} m^2')


if __name__ == '__main__':
    main()
