#!/usr/bin/env python3
"""Render the provenance of the second iteration's scenario.

Three panels: the occupancy map as downloaded, the same map deskewed and
cleaned with the crop window marked, and the extruded Gazebo world with the
fleet's deployment poses. The point of the figure is that the geometry the
agents explore is not authored — it is a map a robot built of a real building,
and the figure shows every step between the two.

    python3 tools/render_scenario.py -o docs/figures/iter2/fig_scenario.png
"""

import argparse
import os
import re
import sys

import numpy as np
from PIL import Image
from scipy import ndimage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import map_to_world as m2w  # noqa: E402
import mrs_style as style  # noqa: E402

style.apply()

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
from matplotlib.ticker import AutoMinorLocator  # noqa: E402


BOX = re.compile(
    r'<pose>(-?[\d.]+) (-?[\d.]+) (-?[\d.]+) 0 0 0</pose>\s*'
    r'<collision name="c"><geometry><box><size>([\d.]+) ([\d.]+) ([\d.]+)</size>')


def raw_occupied(pgm_path, occupied_thresh=0.65):
    """Cells the source map calls occupied, at its own threshold."""
    image = np.array(Image.open(pgm_path))
    if image.ndim != 2:
        image = image[..., 0]
    return (255.0 - image.astype(np.float64)) / 255.0 > occupied_thresh


def world_boxes(path):
    with open(path) as handle:
        return [tuple(float(v) for v in match) for match in BOX.findall(handle.read())]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', default='src/mrs_bringup/maps/willow-2010-02-18-0.10')
    parser.add_argument('--world', default='src/mrs_bringup/worlds/willow_office.world')
    parser.add_argument('--fleet', default='src/mrs_bringup/config/fleet_willow.yaml')
    parser.add_argument('--angle', type=float, default=22.5)
    parser.add_argument('--crop', type=float, nargs=4, default=[-23, -8, 8, 26])
    parser.add_argument('-o', '--output', required=True)
    args = parser.parse_args()

    import yaml
    with open(args.fleet) as handle:
        robots = yaml.safe_load(handle)['fleet']['robots']

    free, res = m2w.load_free_mask(args.map + '.pgm', args.map + '.yaml')
    occupied = raw_occupied(args.map + '.pgm')
    cleaned = m2w.clean(m2w.rotate_mask(free, args.angle), 6, 3)
    rows, cols = np.nonzero(cleaned)
    centre_col, centre_row = cols.mean(), rows.mean()

    # Every panel uses one convention: ink is obstacle, white is free, grey is
    # unobserved. Panel (b) therefore shows the wall band the extrusion will
    # use, not the bare free mask, so (b) and (c) are the same object drawn
    # twice — once as cells, once as collision geometry.
    band_px = max(int(round(0.4 / res)), 1)
    walls = ndimage.binary_dilation(cleaned, m2w.disk(band_px)) & ~cleaned

    fig, axes = plt.subplots(1, 3, figsize=(6.9, 2.75),
                             gridspec_kw=dict(wspace=0.22))

    # (a) as downloaded --------------------------------------------------
    ax = axes[0]
    height, width = free.shape
    classes = np.where(occupied, style.OCCUPIED,
                       np.where(free, style.FREE, style.UNKNOWN))
    ax.imshow(classes, cmap=style.OCC_CMAP, norm=style.OCC_NORM,
              origin='upper', interpolation='nearest',
              extent=[0, width * res, 0, height * res])
    ax.set_title('source map')
    ax.set_xlabel(r'$x$ [m]')
    ax.set_ylabel(r'$y$ [m]')
    style.panel_label(ax, '(a)')

    # (b) deskewed, cleaned, cropped -------------------------------------
    ax = axes[1]
    height, width = cleaned.shape
    extent = [-centre_col * res, (width - centre_col) * res,
              -(height - centre_row) * res, centre_row * res]
    classes = np.where(walls, style.OCCUPIED,
                       np.where(cleaned, style.FREE, style.UNKNOWN))
    ax.imshow(classes, cmap=style.OCC_CMAP, norm=style.OCC_NORM,
              origin='upper', interpolation='nearest', extent=extent)
    x0, y0, x1, y1 = args.crop
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False,
                           edgecolor=style.SERIES[1], lw=1.0, zorder=5))
    ax.set_title(rf'deskewed ${args.angle:.1f}^\circ$, cropped')
    ax.set_xlabel(r'$x$ [m]')
    style.panel_label(ax, '(b)')

    # (c) extruded world with the deployment poses ------------------------
    ax = axes[2]
    boxes = world_boxes(args.world)
    for x, y, _, sx, sy, _ in boxes:
        ax.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy,
                               facecolor='#111111', edgecolor='none'))

    for index, robot in enumerate(robots):
        pose = robot['initial_pose']
        colour = style.SERIES[index % len(style.SERIES)]
        ax.plot([pose['x']], [pose['y']], marker='o', ms=4.5, color=colour,
                zorder=6, linestyle='none')
        ax.annotate('', xy=(pose['x'] + 3.4 * np.cos(pose['yaw']),
                            pose['y'] + 3.4 * np.sin(pose['yaw'])),
                    xytext=(pose['x'], pose['y']), zorder=6,
                    arrowprops=dict(arrowstyle='-|>,head_width=0.16,head_length=0.32',
                                    color=colour, lw=0.9, shrinkA=0, shrinkB=0))
        # Behind the agent, so the label never sits on its heading arrow.
        ax.text(pose['x'] - 1.7 * np.cos(pose['yaw']),
                pose['y'] - 1.7 * np.sin(pose['yaw']),
                rf'${index + 1}$', color=colour, ha='center', va='center',
                fontsize=7.5, zorder=6)

    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_title('extruded world')
    ax.set_xlabel(r'$x$ [m]')
    style.panel_label(ax, '(c)')

    for ax in axes:
        ax.set_aspect('equal')
        ax.xaxis.set_minor_locator(AutoMinorLocator(2))
        ax.yaxis.set_minor_locator(AutoMinorLocator(2))

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    fig.savefig(args.output)
    plt.close(fig)
    print(f'wrote {args.output}  ({len(boxes)} boxes)')


if __name__ == '__main__':
    main()
