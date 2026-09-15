#!/usr/bin/env python3
"""Check sweep results against Gazebo ground truth recorded by gt_logger.py.

For every goal in the sweep CSV: distance from the goal to the true final pose
and to the pose Nav2 believed (TF), the gap between the two, and distance
travelled in truth vs belief. Over the whole trajectory: share of true poses
inside occupied PGM cells (inside a wall).

Usage: gt_analyse.py <label> <sweep_csv> [trials_dir]
Writes <label>.gt_rooms.csv and prints a summary.
"""
import csv, math, os, sys
import numpy as np
import yaml
from PIL import Image

label, sweep_csv = sys.argv[1], sys.argv[2]
tdir = sys.argv[3] if len(sys.argv) > 3 else os.path.expanduser('~/sim_trials')
MAP_YAML = '/home/jason/ros2_ws/src/spatial_maps/maps/1F.yaml'
TOL = 0.5                           # nav2 stopped_goal_checker xy_goal_tolerance


def fnum(v):
    return float(v) if v not in ('', None) else None


def dist(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by) if None not in (ax, ay, bx, by) else None


rows = list(csv.DictReader(open(sweep_csv)))
goal_rows = [r for r in rows if r['result'] not in ('SKIPPED', 'REJECTED')
             and 'accept timed out' not in r['notes']]
events = list(csv.DictReader(open(os.path.join(tdir, f'{label}.gt_events.csv'))))
traj = list(csv.DictReader(open(os.path.join(tdir, f'{label}.gt_traj.csv'))))

terminal = {}
for e in events:
    if e['status'] in ('4', '5', '6'):
        terminal[int(e['goal_seq'])] = e
goals = [terminal[k] for k in sorted(terminal)]

# Align sweep goals with logged goals; the plan end should sit on the sent goal.
def matches(off):
    m = 0
    for i, r in enumerate(goal_rows):
        j = i + off
        if 0 <= j < len(goals):
            d = dist(fnum(goals[j]['plan_end_x']), fnum(goals[j]['plan_end_y']), fnum(r['goal_x']), fnum(r['goal_y']))
            m += d is not None and d < 0.6
    return m
offset = max(range(-5, 6), key=matches)
checked = sum(1 for g in goals if g['plan_end_x'])

# Distance travelled per goal, truth vs belief.
travel = {}
prev = {}
for t in traj:
    if not t['goal_seq'] or not t['gt_x'] or not t['tf_x']:
        continue
    k = int(t['goal_seq'])
    gx, gy, bx, by = map(float, (t['gt_x'], t['gt_y'], t['tf_x'], t['tf_y']))
    if k in prev:
        pg, pb = prev[k]
        a = travel.setdefault(k, [0.0, 0.0])
        a[0] += math.hypot(gx - pg[0], gy - pg[1])
        a[1] += math.hypot(bx - pb[0], by - pb[1])
    prev[k] = ((gx, gy), (bx, by))

out = []
for i, r in enumerate(goal_rows):
    j = i + offset
    g = goals[j] if 0 <= j < len(goals) else None
    gx, gy = fnum(r['goal_x']), fnum(r['goal_y'])
    if g is None:
        out.append({'room': r['room'], 'name': r['display_name'], 'sweep_result': r['result'], 'logged': False})
        continue
    tx, ty, bx, by = (fnum(g[k]) for k in ('gt_x', 'gt_y', 'tf_x', 'tf_y'))
    trav = travel.get(int(g['goal_seq']), [None, None])
    out.append({
        'room': r['room'], 'name': r['display_name'], 'sweep_result': r['result'], 'logged': True,
        'nav2_status': g['status_name'], 'goal_x': gx, 'goal_y': gy,
        'true_x': tx, 'true_y': ty, 'believed_x': bx, 'believed_y': by,
        'true_err_m': dist(tx, ty, gx, gy), 'believed_err_m': dist(bx, by, gx, gy),
        'true_vs_believed_m': dist(tx, ty, bx, by),
        'true_travel_m': trav[0], 'believed_travel_m': trav[1],
        'plan_end_to_goal_m': dist(fnum(g['plan_end_x']), fnum(g['plan_end_y']), gx, gy),
    })

keys = ['room', 'name', 'sweep_result', 'logged', 'nav2_status', 'goal_x', 'goal_y', 'true_x', 'true_y',
        'believed_x', 'believed_y', 'true_err_m', 'believed_err_m', 'true_vs_believed_m',
        'true_travel_m', 'believed_travel_m', 'plan_end_to_goal_m']
with open(os.path.join(tdir, f'{label}.gt_rooms.csv'), 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=keys)
    w.writeheader()
    for o in out:
        w.writerow({k: (f'{v:.3f}' if isinstance(v, float) else v) for k, v in o.items()})

# Trajectory inside walls.
meta = yaml.safe_load(open(MAP_YAML))
img = np.array(Image.open(os.path.join(os.path.dirname(MAP_YAML), meta['image'])))
res, ox, oy = meta['resolution'], meta['origin'][0], meta['origin'][1]
occ = img < (1.0 - meta['occupied_thresh']) * 255
H, W = occ.shape
pts = [(float(t['gt_x']), float(t['gt_y'])) for t in traj if t['gt_x']]
inside = 0
for x, y in pts:
    c, rr = int((x - ox) / res), H - 1 - int((y - oy) / res)
    inside += 0 <= c < W and 0 <= rr < H and occ[rr, c]

succ = [o for o in out if o.get('logged') and o['sweep_result'] == 'SUCCESS' and o['true_err_m'] is not None]
p = f'[{label}]'
print(f'{p} sweep goals {len(goal_rows)}, logged terminal goals {len(goals)}, alignment offset {offset}, '
      f'plan-end matches {matches(offset)}/{checked}')
if succ:
    te = sorted(o['true_err_m'] for o in succ)
    gap = sorted(o['true_vs_believed_m'] for o in succ)
    print(f'{p} sweep SUCCESS with ground truth: {len(succ)}; truly within {TOL} m of goal: '
          f'{sum(e <= TOL for e in te)}; within 1.0 m: {sum(e <= 1.0 for e in te)}; '
          f'median true error {te[len(te) // 2]:.2f} m, max {te[-1]:.2f} m')
    print(f'{p} true vs believed pose at goal end: median {gap[len(gap) // 2]:.2f} m, max {gap[-1]:.2f} m')
    tr = [(o['true_travel_m'], o['believed_travel_m']) for o in succ if o['true_travel_m'] and o['believed_travel_m']]
    if tr:
        print(f'{p} travel during SUCCESS goals: true {sum(a for a, _ in tr):.1f} m vs believed '
              f'{sum(b for _, b in tr):.1f} m (ratio {sum(a for a, _ in tr) / sum(b for _, b in tr):.2f})')
print(f'{p} trajectory samples {len(pts)}, true pose inside occupied PGM cell: {inside} '
      f'({100 * inside / max(len(pts), 1):.1f}%)')
