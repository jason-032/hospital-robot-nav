#!/usr/bin/env python3
"""Derive Nav2 helper maps for a floor from its PGM and BIM network.

  <FLOOR>_05.pgm/.yaml           occupancy map resampled to 0.05 m for the global
                                 costmap (the Smac State Lattice primitives are 5 cm).
                                 Conservative: a coarse cell takes the darkest fine
                                 cell it overlaps, so occupied and unknown win.
  <FLOOR>_speed_mask.pgm/.yaml   Nav2 SpeedFilter mask: discs around the BIM door
                                 nodes limit speed to --door-percent of maximum.
                                 Scale mode, used with base 100 and multiplier -1.

Both keep the source map's origin (bottom-left), so they overlay it exactly.

Usage: make_nav_maps.py <maps dir> <floor e.g. 1F> <network json>
       [--res 0.05] [--door-radius 1.0] [--door-percent 50]
"""
import argparse, json, math, os
import numpy as np
import yaml
from PIL import Image

ap = argparse.ArgumentParser()
ap.add_argument('maps_dir')
ap.add_argument('floor')
ap.add_argument('network_json')
ap.add_argument('--res', type=float, default=0.05)
ap.add_argument('--door-radius', type=float, default=1.0)
ap.add_argument('--door-percent', type=float, default=50.0)
a = ap.parse_args()

meta = yaml.safe_load(open(os.path.join(a.maps_dir, f'{a.floor}.yaml')))
img = np.array(Image.open(os.path.join(a.maps_dir, meta['image'])))
fine = meta['resolution']
ox, oy = meta['origin'][0], meta['origin'][1]
H, W = img.shape
width_m, height_m = W * fine, H * fine
cw, ch = int(math.ceil(width_m / a.res - 1e-9)), int(math.ceil(height_m / a.res - 1e-9))

# Work bottom-up so row 0 is the map origin's row in both grids.
up = img[::-1]
coarse = np.full((ch, cw), 255, np.uint8)
cols = [(int(math.floor(i * a.res / fine + 1e-9)), min(W, int(math.ceil((i + 1) * a.res / fine - 1e-9)))) for i in range(cw)]
rows = [(int(math.floor(j * a.res / fine + 1e-9)), min(H, int(math.ceil((j + 1) * a.res / fine - 1e-9)))) for j in range(ch)]
colmin = np.stack([up[:, c0:c1].min(axis=1) if c1 > c0 else np.full(H, 255, np.uint8) for c0, c1 in cols], axis=1)
for j, (r0, r1) in enumerate(rows):
    if r1 > r0:
        coarse[j] = colmin[r0:r1].min(axis=0)
coarse = coarse[::-1]

base = f'{a.floor}_05'
Image.fromarray(coarse).save(os.path.join(a.maps_dir, f'{base}.pgm'))
out = dict(meta)
out.update({'image': f'{base}.pgm', 'resolution': a.res})
yaml.safe_dump(out, open(os.path.join(a.maps_dir, f'{base}.yaml'), 'w'), default_flow_style=None, sort_keys=False)
occ_f = (img == 0).mean() * 100
occ_c = (coarse == 0).mean() * 100
print(f'{base}: {cw} x {ch} px at {a.res} m (source {W} x {H} at {fine} m); occupied {occ_c:.1f}% (source {occ_f:.1f}%)')

# Speed mask: scale mode with thresholds 0..1, so grey level g gives value 99*(255-g)/255.
net = json.load(open(a.network_json))
doors = [(n['x'], n['y']) for n in net['nodes'] if n.get('type') == 'door']
value = 100.0 - a.door_percent                      # speed_limit = 100 - value
grey = int(round(255 * (1 - value / 99.0)))
mask = np.full((ch, cw), 255, np.uint8)
yy, xx = np.mgrid[0:ch, 0:cw]
cx = ox + (xx + 0.5) * a.res
cy = oy + (ch - 1 - yy + 0.5) * a.res
for dx, dy in doors:
    mask[(cx - dx) ** 2 + (cy - dy) ** 2 <= a.door_radius ** 2] = grey
mname = f'{a.floor}_speed_mask'
Image.fromarray(mask).save(os.path.join(a.maps_dir, f'{mname}.pgm'))
yaml.safe_dump({'image': f'{mname}.pgm', 'mode': 'scale', 'resolution': a.res,
                'origin': [ox, oy, 0.0], 'negate': 0, 'occupied_thresh': 1.0, 'free_thresh': 0.0},
               open(os.path.join(a.maps_dir, f'{mname}.yaml'), 'w'), default_flow_style=None, sort_keys=False)
decoded = 100 - int(99.0 * (255 - grey) / 255.0)
print(f'{mname}: {len(doors)} door discs r={a.door_radius} m, grey {grey} -> speed limit about {decoded}% '
      f'({(mask != 255).mean() * 100:.1f}% of cells limited)')
