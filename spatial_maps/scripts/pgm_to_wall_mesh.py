#!/usr/bin/env python3
"""Utility: occupancy map (PGM + YAML) to a single wall mesh (OBJ) in the map frame.

Every edge between an occupied cell and a cell that is not occupied becomes a
vertical quad facing the open side, and collinear edges are merged into one quad.
The result is the exact outline of the occupied cells, extruded from z = 0 to
--height, as one mesh and therefore one Gazebo link.

Use this when a floor has a map but no BIM wall mesh (a scan-derived map, for
example). It replaces pgm_to_sdf_walls.py for Bullet Featherstone worlds: that
script writes one link per box, and the robot drove through those on 1F (CLAUDE.md
open problem 4), while a single-link mesh does collide.

The mesh is the map by construction, so a simulation built from it tests the
navigation stack on that map. It says nothing about whether the map matches the
real building.

Usage: pgm_to_wall_mesh.py <map.yaml> <out.obj> [--height 1.0]
"""
import argparse
import os

import numpy as np
import yaml
from PIL import Image


def occupied_grid(map_yaml):
    """Return (occ[row = y up, col = x], resolution, origin_x, origin_y)."""
    meta = yaml.safe_load(open(map_yaml))
    img = np.array(Image.open(os.path.join(os.path.dirname(map_yaml), meta['image'])))
    occ = 1.0 - img / 255.0
    if int(meta.get('negate', 0)):
        occ = 1.0 - occ
    occ = np.flipud(occ > float(meta.get('occupied_thresh', 0.65)))
    return occ, float(meta['resolution']), float(meta['origin'][0]), float(meta['origin'][1])


def runs(mask_1d):
    """Start and end (exclusive) of each run of True."""
    d = np.diff(np.concatenate([[0], mask_1d.astype(np.int8), [0]]))
    return zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1))


def wall_quads(occ, res, ox, oy):
    """Yield ((x0, y0), (x1, y1)) wall segments, ordered so the open side is on the left."""
    pad = np.pad(occ, 1, constant_values=False)
    core = pad[1:-1, 1:-1]
    east = core & ~pad[1:-1, 2:]     # open cell to +x
    west = core & ~pad[1:-1, :-2]
    north = core & ~pad[2:, 1:-1]    # open cell to +y
    south = core & ~pad[:-2, 1:-1]
    h, w = occ.shape
    for c in range(w):
        for a, b in runs(east[:, c]):
            x = ox + (c + 1) * res
            yield (x, oy + b * res), (x, oy + a * res)
        for a, b in runs(west[:, c]):
            x = ox + c * res
            yield (x, oy + a * res), (x, oy + b * res)
    for r in range(h):
        for a, b in runs(north[r]):
            y = oy + (r + 1) * res
            yield (ox + a * res, y), (ox + b * res, y)
        for a, b in runs(south[r]):
            y = oy + r * res
            yield (ox + b * res, y), (ox + a * res, y)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('map_yaml')
    ap.add_argument('out_obj')
    ap.add_argument('--height', type=float, default=1.0)
    a = ap.parse_args()

    occ, res, ox, oy = occupied_grid(a.map_yaml)
    quads = list(wall_quads(occ, res, ox, oy))
    with open(a.out_obj, 'w') as f:
        f.write(f'# {os.path.basename(a.map_yaml)}: outline of occupied cells, map frame, '
                f'z 0 to {a.height} m\no walls\n')
        for (x0, y0), (x1, y1) in quads:
            for x, y, z in ((x0, y0, 0.0), (x1, y1, 0.0), (x1, y1, a.height), (x0, y0, a.height)):
                f.write(f'v {x:.4f} {y:.4f} {z:.4f}\n')
        # Walking from p0 to p1 with the open side on the left, the outward normal
        # points left, which this winding gives (counter-clockwise seen from outside).
        for i in range(len(quads)):
            v = 4 * i + 1
            f.write(f'f {v + 1} {v} {v + 3}\nf {v + 1} {v + 3} {v + 2}\n')
    length = sum(abs(x1 - x0) + abs(y1 - y0) for (x0, y0), (x1, y1) in quads)
    print(f'{int(occ.sum())} occupied cells -> {len(quads)} quads, {2 * len(quads)} triangles, '
          f'{length:.0f} m of wall face, written to {a.out_obj}')


if __name__ == '__main__':
    main()
