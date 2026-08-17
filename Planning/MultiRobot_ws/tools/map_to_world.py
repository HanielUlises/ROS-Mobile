#!/usr/bin/env python3
"""Extrude a real 2D occupancy map into a Gazebo world.

Consumes a `map_server` pair (`<map>.pgm` + `<map>.yaml`) and emits an SDF world
whose walls reproduce the mapped building. The source used by this project is
the Willow Garage office map recorded in 2010 and distributed with
`turtlebot_navigation`; it is a map a real robot built with a real laser in a
real building, which is the property that matters here — its corridors close
into loops, its rooms are unequal, and none of its geometry was chosen to
flatter a scan matcher.

Pipeline
--------

1. **Binarise.** Only cells the source map calls *free* are taken as free. The
   probabilistic grey band between free and unknown is discarded rather than
   thresholded into obstacles: it is mostly range-return spray along walls, and
   admitting it thickens every wall by an amount that varies with how often the
   original robot looked at it.

2. **Deskew.** The source map is recorded in the mapping robot's start frame, so
   the building sits at roughly 23 deg to the raster. The rotation that maximises
   the row/column mass concentration of the free mask is applied, which puts the
   corridors back on the axes. This costs nothing physically — it only chooses
   the world frame — and it is what lets step 5 cover long straight walls with
   few boxes instead of a stair of many.

3. **Clean.** A morphological opening removes the thin radial spikes where the
   original laser saw through doorways and windows into unmapped space; a
   closing seals pinholes inside rooms; the largest connected component is kept
   so the world is a single navigable region. Without the opening, the spikes
   become tunnels leading out of the building and the reactive explorer leaves
   through them.

4. **Wall band.** Walls are the cells within `band` metres of free space that
   are not themselves free. Extruding *all* non-free cells instead would be
   correct but wasteful: the unmapped building core is a solid block of tens of
   thousands of cells that no laser can ever reach past its surface.

5. **Rectangle cover.** The wall mask is covered greedily by maximal axis-aligned
   rectangles (longest horizontal run, extended down while the run stays clear).
   Each rectangle becomes one box link. The cover is what keeps the collision
   count in the hundreds rather than the tens of thousands, which matters because
   the lidar is raycast on the CPU.

    python3 tools/map_to_world.py --crop -14 -12 14 12 -o warehouse_willow.world
"""

import argparse
import os
import sys

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage


# --------------------------------------------------------------------------
# map ingest
# --------------------------------------------------------------------------

def load_free_mask(pgm_path, yaml_path):
    """Return (free mask, resolution) for a map_server map pair."""
    with open(yaml_path) as handle:
        meta = yaml.safe_load(handle)

    image = np.array(Image.open(pgm_path))
    if image.ndim != 2:
        image = image[..., 0]

    # map_server occupancy convention: p = (255 - value) / 255, so a cell is
    # free when its probability falls below free_thresh.
    prob = (255.0 - image.astype(np.float64)) / 255.0
    free = prob < float(meta.get('free_thresh', 0.196))
    return free, float(meta['resolution'])


def best_rotation(free, step=0.5):
    """Angle (deg) that puts the building's dominant walls on the raster axes.

    Scored by the sum of squared row and column masses, which is maximal when
    long straight structure lines up with a single row or column rather than
    being smeared across many.
    """
    source = Image.fromarray((free * 255).astype(np.uint8))
    best_angle, best_score = 0.0, -np.inf
    for angle in np.arange(0.0, 90.0, step):
        rotated = np.array(source.rotate(angle, resample=Image.NEAREST, expand=True)) > 127
        score = float((rotated.sum(0).astype(np.float64) ** 2).sum() +
                      (rotated.sum(1).astype(np.float64) ** 2).sum())
        if score > best_score:
            best_angle, best_score = float(angle), score
    return best_angle


def rotate_mask(free, angle):
    source = Image.fromarray((free * 255).astype(np.uint8))
    return np.array(source.rotate(angle, resample=Image.BILINEAR, expand=True)) > 127


def clean(free, open_px, close_px):
    """Strip laser spray, seal pinholes, keep one connected region."""
    if open_px > 0:
        free = ndimage.binary_opening(free, disk(open_px))
    if close_px > 0:
        free = ndimage.binary_closing(free, disk(close_px))

    labels, count = ndimage.label(free)
    if count > 1:
        sizes = ndimage.sum(free, labels, range(1, count + 1))
        free = labels == (int(np.argmax(sizes)) + 1)

    # Interior holes left by furniture are genuine obstacles and are kept; only
    # holes smaller than the sensor's own footprint are filled, since a hole the
    # robot cannot enter is indistinguishable from solid but costs boxes.
    holes = ndimage.binary_fill_holes(free) & ~free
    labels, count = ndimage.label(holes)
    if count:
        sizes = ndimage.sum(holes, labels, range(1, count + 1))
        tiny = np.isin(labels, 1 + np.nonzero(sizes < 4)[0])
        free = free | tiny
    return free


def disk(radius):
    span = np.arange(-radius, radius + 1)
    xs, ys = np.meshgrid(span, span)
    return (xs ** 2 + ys ** 2) <= radius ** 2 + 1e-9


# --------------------------------------------------------------------------
# geometry extraction
# --------------------------------------------------------------------------

def seal_border(free, band_px):
    """Withdraw the outer ring from the free region so it can be walled off.

    A crop cuts corridors open. The ring has to be taken out of `free` rather
    than merely added to the walls, because the wall band is defined as the
    non-free cells near free ones and would otherwise exclude exactly the cells
    that need to cap the severed corridor — which is how a robot ends up
    driving out of the building through the crop boundary.
    """
    free = free.copy()
    free[:band_px, :] = free[-band_px:, :] = False
    free[:, :band_px] = free[:, -band_px:] = False
    return free


def wall_mask(free, band_px):
    """Non-free cells within `band_px` of free space, plus a sealed border."""
    walls = ndimage.binary_dilation(free, disk(band_px)) & ~free
    walls[:band_px, :] = walls[-band_px:, :] = True
    walls[:, :band_px] = walls[:, -band_px:] = True
    return walls


def rectangle_cover(mask):
    """Greedy maximal-rectangle cover of a boolean mask.

    Yields (row0, col0, height, width). Greedy is enough: the cover only needs
    to be small, not minimal, and the optimal decomposition of a rectilinear
    polygon is not worth its cost here.
    """
    remaining = mask.copy()
    height, width = mask.shape
    rects = []

    for row in range(height):
        col = 0
        while col < width:
            if not remaining[row, col]:
                col += 1
                continue

            end = col
            while end < width and remaining[row, end]:
                end += 1
            run = slice(col, end)

            bottom = row + 1
            while bottom < height and remaining[bottom, run].all():
                bottom += 1

            remaining[row:bottom, run] = False
            rects.append((row, col, bottom - row, end - col))
            col = end

    return rects


# --------------------------------------------------------------------------
# SDF emission
# --------------------------------------------------------------------------

WORLD_TEMPLATE = """<?xml version="1.0" ?>
<!--
  GENERATED by tools/map_to_world.py - do not edit by hand.

  Source map : {source}
  Provenance : Willow Garage office, mapped 2010, distributed with
               turtlebot_navigation (BSD). Deskewed by {angle:.1f} deg and cropped
               to {extent}.
  Geometry   : {n_boxes} box links covering {wall_area:.1f} m2 of wall,
               enclosing {free_area:.1f} m2 of navigable floor at {res:.3f} m/cell.
-->
<sdf version="1.6">
  <world name="{name}">

    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>{rtf}</real_time_factor>
      <real_time_update_rate>{rate}</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <shadows>false</shadows>
    </scene>

    <model name="{name}_walls">
      <static>true</static>
{links}    </model>

  </world>
</sdf>
"""

LINK_TEMPLATE = """      <link name="w{index:04d}">
        <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
        <collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material><ambient>0.68 0.66 0.62 1</ambient><diffuse>0.68 0.66 0.62 1</diffuse></material>
        </visual>
      </link>
"""


def emit_world(rects, res, origin_xy, height, name, source, angle, extent,
               free_area, rtf, rate):
    ox, oy = origin_xy
    links = []
    wall_area = 0.0
    for index, (row, col, rows, cols) in enumerate(rects):
        sx, sy = cols * res, rows * res
        wall_area += sx * sy
        links.append(LINK_TEMPLATE.format(
            index=index,
            x=ox + (col + cols / 2.0) * res,
            y=oy + (row + rows / 2.0) * res,
            z=height / 2.0,
            sx=sx, sy=sy, sz=height))

    return WORLD_TEMPLATE.format(
        name=name, source=source, angle=angle, extent=extent,
        n_boxes=len(rects), wall_area=wall_area, free_area=free_area, res=res,
        rtf=rtf, rate=rate, links=''.join(links))


# --------------------------------------------------------------------------

def main(argv=None):
    here = os.path.dirname(os.path.abspath(__file__))
    default_map = os.path.join(here, os.pardir, 'src', 'mrs_bringup', 'maps',
                               'willow-2010-02-18-0.10')

    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--map', default=default_map,
                        help='map stem, without the .pgm/.yaml extension')
    parser.add_argument('--name', default='willow', help='SDF world name')
    parser.add_argument('-o', '--output', required=True, help='destination .world file')
    parser.add_argument('--angle', type=float, default=None,
                        help='deskew angle in degrees (default: estimated)')
    parser.add_argument('--crop', type=float, nargs=4, default=None,
                        metavar=('X0', 'Y0', 'X1', 'Y1'),
                        help='crop window in metres, in the deskewed frame, '
                             'measured from the centre of the mapped region')
    parser.add_argument('--band', type=float, default=0.4,
                        help='wall thickness in metres')
    parser.add_argument('--height', type=float, default=1.0, help='wall height in metres')
    parser.add_argument('--open', type=int, default=3, dest='open_px',
                        help='opening radius in cells (spike removal)')
    parser.add_argument('--close', type=int, default=2, dest='close_px',
                        help='closing radius in cells (pinhole sealing)')
    parser.add_argument('--rtf', type=float, default=1.0, help='target real time factor')
    parser.add_argument('--rate', type=float, default=250.0, help='real time update rate')
    parser.add_argument('--preview', default=None, help='write a PNG of the extruded mask')
    args = parser.parse_args(argv)

    free, res = load_free_mask(args.map + '.pgm', args.map + '.yaml')

    angle = args.angle if args.angle is not None else best_rotation(free)
    free = clean(rotate_mask(free, angle), args.open_px, args.close_px)

    # Origin at the centroid of the navigable region, so crop windows and robot
    # deployment poses are stated in a frame anchored to the building rather
    # than to the corner of whatever raster the source map happened to use.
    rows, cols = np.nonzero(free)
    centre_col, centre_row = cols.mean(), rows.mean()

    if args.crop:
        x0, y0, x1, y1 = args.crop
        c0 = int(round(centre_col + x0 / res))
        c1 = int(round(centre_col + x1 / res))
        r0 = int(round(centre_row - y1 / res))
        r1 = int(round(centre_row - y0 / res))
        c0, r0 = max(c0, 0), max(r0, 0)
        c1, r1 = min(c1, free.shape[1]), min(r1, free.shape[0])
        free = free[r0:r1, c0:c1]
        centre_col -= c0
        centre_row -= r0
        # Cropping can sever the region; keep the part that survives.
        free = clean(free, 0, 0)
        extent = f'{(c1 - c0) * res:.1f} x {(r1 - r0) * res:.1f} m'
    else:
        extent = f'{free.shape[1] * res:.1f} x {free.shape[0] * res:.1f} m'

    band_px = max(int(round(args.band / res)), 1)
    free = clean(seal_border(free, band_px), 0, 0)
    walls = wall_mask(free, band_px)

    # World y grows upward, the raster's row index grows downward, so the mask
    # is flipped before its rows are read as metres.
    walls_world = np.flipud(walls)
    free_world = np.flipud(free)
    rects = rectangle_cover(walls_world)
    origin = (-centre_col * res, -(free.shape[0] - 1 - centre_row) * res)

    world = emit_world(
        rects, res, origin, args.height, args.name,
        os.path.basename(args.map) + '.pgm', angle, extent,
        float(free.sum()) * res * res, args.rtf, args.rate)

    with open(args.output, 'w') as handle:
        handle.write(world)

    if args.preview:
        canvas = np.full(free.shape + (3,), 232, dtype=np.uint8)
        canvas[free_world] = 255
        canvas[walls_world] = 32
        Image.fromarray(canvas).save(args.preview)

    print(f'{args.output}: {len(rects)} boxes, {extent}, '
          f'{free.sum() * res * res:.1f} m2 navigable, deskew {angle:.1f} deg')
    return 0


if __name__ == '__main__':
    sys.exit(main())
