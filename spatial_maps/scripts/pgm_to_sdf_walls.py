#!/usr/bin/env python3
"""
Generate Gazebo SDF collision walls from a ROS2 PGM occupancy map.

Scans every boundary between an occupied cell and a free cell (horizontal and
vertical edges), merges collinear adjacent boundaries into longer run segments,
then writes an SDF model with one collision-only box per run.  The resulting
geometry is pixel-perfectly aligned with the costmap so AMCL can localise.

Usage:
    python3 pgm_to_sdf_walls.py \
        --yaml  path/to/floor.yaml \
        --output path/to/output_walls.sdf \
        [--wall-height 3.0] [--wall-thickness 0.05] \
        [--min-length 0.06] [--downsample 3]
"""

import argparse
import os
import sys
import time
import numpy as np
import yaml


# ---------------------------------------------------------------------------
# PGM loader
# ---------------------------------------------------------------------------

def load_pgm(path: str):
    """Return (array[H,W] uint8, W, H, maxval)."""
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        W, H = map(int, line.split())
        maxval = int(f.readline().strip())
        raw = f.read()
    if magic == b'P5':
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(H, W)
    elif magic == b'P2':
        arr = np.array(list(map(int, raw.split())), dtype=np.uint8).reshape(H, W)
    else:
        raise ValueError(f'Unknown PGM format: {magic}')
    return arr, W, H, maxval


# ---------------------------------------------------------------------------
# Main generator
# ---------------------------------------------------------------------------

def generate_walls(yaml_path: str, output_path: str,
                   wall_height: float = 3.0,
                   wall_thickness: float = 0.05,
                   min_length: float = 0.06,
                   downsample: int = 3):
    """
    Parse the PGM referenced by yaml_path and write an SDF collision model.

    Parameters
    ----------
    downsample  Factor by which to reduce resolution before scanning.
                3 → 0.06 m/cell (fast, ~2k–5k segments for a hospital floor).
    min_length  Skip boundary runs shorter than this (metres) — removes
                single-pixel noise at diagonal wall corners.
    """
    t0 = time.perf_counter()

    # Load map metadata
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    pgm_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
    res_orig  = float(meta['resolution'])
    origin    = meta['origin']
    ox, oy    = float(origin[0]), float(origin[1])
    occ_thresh = float(meta.get('occupied_thresh', 0.65))

    # Load PGM
    pixels, W0, H0, maxval = load_pgm(pgm_path)
    print(f'Loaded {pgm_path}  ({W0}×{H0} px, {res_orig} m/px)')

    # Downsample with min-pooling (occupied if ANY sub-pixel occupied)
    if downsample > 1:
        Hd = (H0 // downsample) * downsample
        Wd = (W0 // downsample) * downsample
        pixels = pixels[:Hd, :Wd]
        H = Hd // downsample
        W = Wd // downsample
        # min over each downsample×downsample block → smallest pixel value
        # (smallest pixel = most occupied in ROS convention 0=occ, 255=free)
        pixels = pixels.reshape(H, downsample, W, downsample).min(axis=(1, 3))
        res = res_orig * downsample
    else:
        H, W = H0, W0
        res = res_orig
    print(f'Working at {W}×{H} px  ({res:.3f} m/px)')

    # Occupied mask  — True = occupied
    # ROS convention: occ = 1 - pixel/maxval;  occupied if occ > occupied_thresh
    occ = (1.0 - pixels.astype(np.float32) / maxval) > occ_thresh

    # PGM coordinate convention (row 0 = TOP of image = highest world-y):
    #   world_x(col)  = ox + col * res
    #   world_y(row)  = oy + (H - 1 - row) * res

    walls = []   # (cx, cy, length, is_horizontal)

    # ------------------------------------------------------------------
    # Horizontal boundary runs: between row r (above) and row r+1 (below)
    # Wall centre y  = oy + (H - 1 - r)*res  -  res/2
    #               = oy + (H - 1.5 - r) * res
    # ------------------------------------------------------------------
    above = occ[:-1, :]   # rows 0..H-2
    below = occ[1:,  :]   # rows 1..H-1
    h_bnd = above ^ below  # True where there is a boundary

    for r in range(H - 1):
        wy  = oy + (H - 1.5 - r) * res
        row = h_bnd[r]
        c   = 0
        while c < W:
            if row[c]:
                c0 = c
                while c < W and row[c]:
                    c += 1
                length = (c - c0) * res
                if length >= min_length:
                    cx = ox + (c0 + c - 1) / 2.0 * res
                    walls.append((cx, wy, length, True))
            else:
                c += 1

    # ------------------------------------------------------------------
    # Vertical boundary runs: between col c (left) and col c+1 (right)
    # Wall centre x = ox + (c + 0.5) * res
    # ------------------------------------------------------------------
    left  = occ[:, :-1]  # cols 0..W-2
    right = occ[:, 1:]   # cols 1..W-1
    v_bnd = left ^ right

    for c in range(W - 1):
        wx  = ox + (c + 0.5) * res
        col = v_bnd[:, c]
        r   = 0
        while r < H:
            if col[r]:
                r0 = r
                while r < H and col[r]:
                    r += 1
                length = (r - r0) * res
                if length >= min_length:
                    # y_top    = oy + (H - 1 - r0)   * res
                    # y_bottom = oy + (H - 1 - (r-1)) * res = oy + (H - r) * res
                    # centre_y = (y_top + y_bottom) / 2
                    cy = oy + (2*H - 1 - r0 - r) / 2.0 * res
                    walls.append((wx, cy, length, False))
            else:
                r += 1

    n_h = sum(1 for *_, h in walls if h)
    n_v = len(walls) - n_h
    print(f'Wall segments: {n_h} horizontal  +  {n_v} vertical  =  {len(walls)} total')
    print(f'Scan took {time.perf_counter()-t0:.1f}s')

    # ------------------------------------------------------------------
    # Write SDF
    # ------------------------------------------------------------------
    lines = [
        '<?xml version="1.0"?>',
        '<sdf version="1.10">',
        f'  <!-- Auto-generated by pgm_to_sdf_walls.py from {os.path.basename(yaml_path)} -->',
        f'  <!-- {len(walls)} collision boxes  |  wall_height={wall_height}m  |  downsample={downsample}  -->',
        '  <model name="pgm_walls_1f">',
        '    <static>true</static>',
        '    <pose>0 0 0 0 0 0</pose>',
    ]
    z_center = wall_height / 2.0
    for i, (cx, cy, length, horiz) in enumerate(walls):
        if horiz:
            sx, sy = f'{length:.4f}', f'{wall_thickness:.4f}'
        else:
            sx, sy = f'{wall_thickness:.4f}', f'{length:.4f}'
        lines += [
            f'    <link name="w{i}">',
            f'      <pose>{cx:.4f} {cy:.4f} {z_center:.3f} 0 0 0</pose>',
            f'      <collision name="col">',
            f'        <geometry><box><size>{sx} {sy} {wall_height:.3f}</size></box></geometry>',
            f'      </collision>',
            f'    </link>',
        ]
    lines += ['  </model>', '</sdf>']

    with open(output_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')

    size_kb = os.path.getsize(output_path) / 1024
    print(f'SDF written → {output_path}  ({size_kb:.0f} KB)')
    return len(walls)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--yaml',      required=True, help='Path to ROS2 map YAML')
    p.add_argument('--output',    required=True, help='Output SDF path')
    p.add_argument('--wall-height',     type=float, default=3.0)
    p.add_argument('--wall-thickness',  type=float, default=0.05)
    p.add_argument('--min-length',      type=float, default=0.06,
                   help='Minimum wall segment length in metres (default 0.06)')
    p.add_argument('--downsample',      type=int,   default=3,
                   help='Resolution reduction factor (default 3 → 0.06 m/px)')
    args = p.parse_args()

    generate_walls(
        yaml_path=args.yaml,
        output_path=args.output,
        wall_height=args.wall_height,
        wall_thickness=args.wall_thickness,
        min_length=args.min_length,
        downsample=args.downsample,
    )


if __name__ == '__main__':
    main()
