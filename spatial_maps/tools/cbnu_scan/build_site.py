#!/usr/bin/env python3
"""Build the simulation site for the CBNU scan-derived Nav2 map.

Input is the cbnu-scan2bim progress bundle (release cbnu-progress-2026-09-29 of
jason-032/claude-workspace, unzipped): out/nav2_map/cbnu_scan_map.{pgm,yaml} and
data/*_3cm.npy. Writes, inside this package:

  maps/CBNU_scan.{pgm,yaml}          the scan map, unchanged
  maps/CBNU_scan_speed_mask.*        all zero: no speed limit anywhere
  meshes/cbnu_scan_walls.obj         outline of the occupied cells (pgm_to_wall_mesh.py)
  worlds/cbnu_scan.world             ground plane, wall mesh, perimeter fence
  config/gz_bridge_cbnu_scan.yaml    the 1F bridge with the world name changed
  config/cbnu_scan_rooms.json        the 11 scanned spaces as sweep goals
  config/cbnu_scan_site.yaml         spawn pose and map extent

Room centres are the median x, y of each space's points in the map frame, the
definition occupancy.py in the bundle uses. The map frame is the scanner frame
rotated by the angle in the map YAML comment.

Usage: build_site.py <bundle dir containing out/ and data/>
"""
import json
import math
import os
import re
import shutil
import subprocess
import sys

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

PKG = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..'))
bundle = sys.argv[1]
SITE = 'CBNU_scan'
SPAWN_CLEARANCE = 0.85      # the clearance sweep_test.py projects goals to
SPAWN_ROOM = 'HallWay_3'

src_yaml = os.path.join(bundle, 'out', 'nav2_map', 'cbnu_scan_map.yaml')
text = open(src_yaml).read()
meta = yaml.safe_load(text)
rot_deg = float(re.search(r'rotated by (-?[\d.]+) deg', text).group(1))
res, (ox, oy) = float(meta['resolution']), meta['origin'][:2]

# ── map ──────────────────────────────────────────────────────────────────────
shutil.copy(os.path.join(bundle, 'out', 'nav2_map', meta['image']), f'{PKG}/maps/{SITE}.pgm')
# The bundle's YAML has free_thresh 0.25, which turns the unknown grey (205, occupancy
# 0.19608) into free space in nav2_map_server and in this package's map publisher.
# 0.196 keeps it unknown, as on the hospital maps.
assert float(meta['free_thresh']) > 0.19608, 'upstream map fixed: drop this patch'
open(f'{PKG}/maps/{SITE}.yaml', 'w').write(
    text.replace(meta['image'], f'{SITE}.pgm').replace(f"free_thresh: {meta['free_thresh']}", 'free_thresh: 0.196'))
img = np.array(Image.open(f'{PKG}/maps/{SITE}.pgm'))
h, w = img.shape
Image.fromarray(np.full((h, w), 255, np.uint8)).save(f'{PKG}/maps/{SITE}_speed_mask.pgm')
open(f'{PKG}/maps/{SITE}_speed_mask.yaml', 'w').write(
    f'image: {SITE}_speed_mask.pgm\nmode: scale\nresolution: {res}\n'
    f'origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 1.0\nfree_thresh: 0.0\n')

# ── wall mesh ────────────────────────────────────────────────────────────────
mesh = f'{PKG}/meshes/cbnu_scan_walls.obj'
subprocess.run([sys.executable, f'{PKG}/scripts/pgm_to_wall_mesh.py', f'{PKG}/maps/{SITE}.yaml', mesh],
               check=True)

# ── rooms ────────────────────────────────────────────────────────────────────
t = math.radians(rot_deg)
R = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
rooms = sorted(f[:-8] for f in os.listdir(f'{bundle}/data') if f.endswith('_3cm.npy'))
centre, lo = {}, []
for r in rooms:
    p = np.load(f'{bundle}/data/{r}_3cm.npy')[:, :2] @ R.T
    centre[r] = np.median(p, 0)
    lo.append(p.min(0))
check = np.min(lo, 0) - 0.5
print(f'origin recomputed from the points ({check[0]:.3f}, {check[1]:.3f}) vs map YAML ({ox}, {oy})')
assert np.abs(check - [ox, oy]).max() < 0.05, 'room points and map are not in the same frame'
entities = {r: {'guid': r, 'name': r, 'ifc_type': 'IfcSpace', 'storey_name': SITE, 'has_geometry': True,
                'geometry': {'centroid': [round(float(centre[r][0]), 3), round(float(centre[r][1]), 3), 0.0]},
                'properties': {'Identity Data': {'Name': r}}}
            for r in rooms}
json.dump({'entities': entities}, open(f'{PKG}/config/cbnu_scan_rooms.json', 'w'), indent=1)

# ── spawn: nearest cell to the spawn room centre with enough clearance ───────
free = np.flipud(img == 254)
dist = ndimage.distance_transform_edt(free) * res
ok = np.argwhere(dist >= SPAWN_CLEARANCE)
c = centre[SPAWN_ROOM]
xy = np.stack([ox + (ok[:, 1] + 0.5) * res, oy + (ok[:, 0] + 0.5) * res], 1)
sx, sy = (float(v) for v in xy[np.argmin(((xy - c) ** 2).sum(1))])
# Face along the longer free direction so the first move is not into a wall.
r0, c0 = int((sy - oy) / res), int((sx - ox) / res)
run_x = int(np.argmin(np.r_[free[r0, c0:], False])) + int(np.argmin(np.r_[free[r0, c0::-1], False]))
run_y = int(np.argmin(np.r_[free[r0:, c0], False])) + int(np.argmin(np.r_[free[r0::-1, c0], False]))
yaw = 0.0 if run_x >= run_y else math.pi / 2
site = {'spawn_x': round(sx, 3), 'spawn_y': round(sy, 3), 'spawn_yaw': round(yaw, 4),
        'origin': [ox, oy], 'size_m': [round(w * res, 2), round(h * res, 2)], 'frame_rotation_deg': rot_deg}
yaml.safe_dump(site, open(f'{PKG}/config/cbnu_scan_site.yaml', 'w'))
print('site', site)

# ── bridge and world ─────────────────────────────────────────────────────────
open(f'{PKG}/config/gz_bridge_cbnu_scan.yaml', 'w').write(
    open(f'{PKG}/config/gz_bridge_1f.yaml').read().replace('/world/1f/', '/world/cbnu_scan/'))
lx, ly = w * res, h * res
cx, cy = ox + lx / 2, oy + ly / 2


def fence(name, sx_, sy_, px, py):
    return (f'    <model name="fence_{name}"><static>true</static><link name="link">'
            f'<collision name="col"><geometry><box><size>{sx_:.2f} {sy_:.2f} 3.0</size></box></geometry>'
            f'</collision></link><pose>{px:.3f} {py:.3f} 1.5 0 0 0</pose></model>\n')


world = open(f'{PKG}/worlds/1f.world').read()
head = world[:world.index('<!-- Floor plan ground plane')]
head = head.replace('<world name="1f">', '<world name="cbnu_scan">')
body = f'''<!-- CBNU scan-derived map. Generated by tools/cbnu_scan/build_site.py, do not edit. -->
    <!-- The walls are the outline of the occupied cells of maps/{SITE}.pgm, so this world -->
    <!-- is the map by construction. It tests the navigation stack on the map and says     -->
    <!-- nothing about how well the map matches the building.                              -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{lx:.2f} {ly:.2f}</size></plane></geometry>
          <surface>
            <friction>
              <!-- Bullet Featherstone needs torsional 0 here or the robot cannot yaw (CLAUDE.md). -->
              <torsional><coefficient>0</coefficient></torsional>
              <ode><mu>100</mu><mu2>50</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>{lx:.2f} {ly:.2f}</size></plane></geometry>
          <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material>
        </visual>
      </link>
      <pose>{cx:.3f} {cy:.3f} -0.01 0 0 0</pose>
    </model>
    <model name="cbnu_scan_walls">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><mesh><uri>file://{mesh}</uri></mesh></geometry>
        </collision>
        <visual name="visual">
          <geometry><mesh><uri>file://{mesh}</uri></mesh></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>
      </link>
      <pose>0 0 0 0 0 0</pose>
    </model>
    <!-- Invisible perimeter fence 1 m outside each map edge, as on 1F. -->
'''
body += fence('west', 0.2, ly + 2.4, ox - 1.0, cy) + fence('east', 0.2, ly + 2.4, ox + lx + 1.0, cy)
body += fence('south', lx + 2.4, 0.2, cx, oy - 1.0) + fence('north', lx + 2.4, 0.2, cx, oy + ly + 1.0)
open(f'{PKG}/worlds/cbnu_scan.world', 'w').write(head + body + '  </world>\n</sdf>\n')
print('wrote world, bridge, rooms, maps')
