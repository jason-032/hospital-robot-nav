#!/usr/bin/env python3
"""Offline design experiment: how many 1F sweep goals can a robot design reach?

For each candidate footprint (polygon in the robot frame, origin at the rotation
centre, which for a differential drive is the wheel-axle midpoint) and each
planning model, computes reachability from spawn on the PGM:

  circle  the robot needs clearance for its swept circle everywhere (a holonomic
          planner such as NavFn with an honest radius)
  se2     the real footprint is checked at 8 headings; the robot translates along
          its heading and rotates in place only where the swept rotation is free
          (what a footprint-aware planner such as Smac State Lattice can exploit)

A goal counts as reachable if a reached pose lies within the goal tolerance.
Also reports passage width at the 1F door nodes of the BIM network.

Usage: footprint_reachability.py <sweep_csv> [resolution_m]
"""
import csv, json, math, sys
import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

SRC = '/home/jason/ros2_ws/src/spatial_maps'
SWEEP = sys.argv[1]
RES = float(sys.argv[2]) if len(sys.argv) > 2 else 0.04
SPAWN = (21.0, 38.0)
TOL = 0.5
NH = 8


def rect(x0, x1, y0, y1):
    return (x0, x1, y0, y1)


# Footprints as unions of rectangles (x forward, y left), origin at the axle midpoint.
DESIGNS = {
    'D0 current robot (axle 0.15 m behind body centre)': [
        rect(-0.15, 0.45, -0.20, 0.20), rect(-0.10, 0.10, -0.25, 0.25), rect(0.45, 0.46, -0.05, 0.05)],
    'D1 same body, axle at body centre': [
        rect(-0.30, 0.30, -0.20, 0.20), rect(-0.10, 0.10, -0.25, 0.25), rect(0.30, 0.31, -0.05, 0.05)],
    'D2 axle centred, wheels inside 0.40 m width': [
        rect(-0.30, 0.31, -0.20, 0.20)],
    'D3 compact 0.50 x 0.36 m, axle centred': [
        rect(-0.25, 0.25, -0.18, 0.18)],
    'D4 round, diameter 0.50 m': 'circle:0.25',
    'D5 round, diameter 0.40 m': 'circle:0.20',
}


def load_map():
    meta = yaml.safe_load(open(f'{SRC}/maps/1F.yaml'))
    img = np.array(Image.open(f'{SRC}/maps/{meta["image"]}'))
    free = img > (1.0 - meta['free_thresh']) * 255          # anything not clearly free blocks
    f = max(1, int(round(RES / meta['resolution'])))
    h, w = free.shape[0] // f * f, free.shape[1] // f * f
    free = free[free.shape[0] - h:, :w]                        # keep the bottom-left origin aligned
    coarse = free.reshape(h // f, f, w // f, f).all(axis=(1, 3))
    res = meta['resolution'] * f
    return coarse, res, meta['origin'][0], meta['origin'][1]


def to_rc(x, y, res, ox, oy, H):
    return H - 1 - int((y - oy) / res), int((x - ox) / res)


def kernel(design, headings, res):
    """Rasterised union of the footprint over the given headings (radians)."""
    if isinstance(design, str):
        r = float(design.split(':')[1])
        n = int(math.ceil(r / res))
        yy, xx = np.mgrid[-n:n + 1, -n:n + 1]
        return (np.hypot(xx, yy) * res <= r + 1e-9).astype(np.uint8)
    ext = max(math.hypot(max(abs(a), abs(b)), max(abs(c), abs(d))) for a, b, c, d in design)
    n = int(math.ceil(ext / res)) + 1
    k = np.zeros((2 * n + 1, 2 * n + 1), np.uint8)
    jj, ii = np.meshgrid(np.arange(-n, n + 1), np.arange(-n, n + 1))
    wx, wy = jj * res, -ii * res                                 # cell centre offsets in world axes
    for th in headings:
        c, s = math.cos(th), math.sin(th)
        bx, by = c * wx + s * wy, -s * wx + c * wy               # world offset into robot frame
        for x0, x1, y0, y1 in design:
            # a cell is covered if its centre is within the rectangle grown by half a cell
            k |= ((bx >= x0 - res / 2) & (bx <= x1 + res / 2) & (by >= y0 - res / 2) & (by <= y1 + res / 2)).astype(np.uint8)
    return k


def collides(blocked, k):
    return ndimage.correlate(blocked.astype(np.uint8), k, mode='constant', cval=1) > 0


def line_labels(mask, step):
    di, dj = step
    st = np.zeros((3, 3), bool)
    st[1, 1] = st[1 + di, 1 + dj] = st[1 - di, 1 - dj] = True
    lab, n = ndimage.label(mask, structure=st)
    return lab, n


def grow(lab, n, seed):
    ids = np.zeros(n + 1, bool)
    ids[np.unique(lab[seed])] = True
    ids[0] = False
    return ids[lab]


def main():
    free, res, ox, oy = load_map()
    H, W = free.shape
    blocked = ~free
    rows = [r for r in csv.DictReader(open(SWEEP)) if r['result'] not in ('SKIPPED',) and r['goal_x']]
    goals = [(r['room'], r['display_name'], float(r['goal_x']), float(r['goal_y'])) for r in rows if r['room'] != 'RECOVERY']
    sr, sc = to_rc(*SPAWN, res, ox, oy, H)
    dist = ndimage.distance_transform_edt(free) * res

    # door passage widths
    net = json.load(open(f'{SRC}/network/network_1f.json'))
    widths = []
    for nd in net['nodes']:
        if nd['type'] != 'door':
            continue
        r, c = to_rc(nd['x'], nd['y'], res, ox, oy, H)
        rad = int(0.15 / res)
        win = dist[max(r - rad, 0):r + rad + 1, max(c - rad, 0):c + rad + 1]
        widths.append(2 * win.max() if win.size else 0.0)
    w = np.array(sorted(widths))
    print(f'1F door nodes {len(w)}: passage width (2 x clearance) median {np.median(w):.2f} m, '
          f'p10 {np.percentile(w, 10):.2f}, min {w.min():.2f}; '
          f'< 0.70 m: {(w < 0.70).sum()}, < 0.90 m: {(w < 0.90).sum()}, < 1.10 m: {(w < 1.10).sum()}')

    near = {}
    for name, _, gx, gy in goals:
        gr, gc = to_rc(gx, gy, res, ox, oy, H)
        n = int(TOL / res)
        yy, xx = np.mgrid[-n:n + 1, -n:n + 1]
        disk = np.hypot(xx, yy) * res <= TOL
        near[name] = (gr, gc, n, disk)

    def reached_goals(reach2d):
        out = []
        for name, (gr, gc, n, disk) in near.items():
            r0, r1, c0, c1 = gr - n, gr + n + 1, gc - n, gc + n + 1
            if r0 < 0 or c0 < 0 or r1 > H or c1 > W:
                continue
            if (reach2d[r0:r1, c0:c1] & disk).any():
                out.append(name)
        return set(out)

    steps = [(0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1), (1, 0), (1, 1)]   # (drow, dcol) for 0,45,...,315 deg
    results = {}
    for dname, design in DESIGNS.items():
        # circle model
        full = kernel(design, [k * math.pi / 36 for k in range(72)], res)
        circ_free = ~collides(blocked, full)
        lab, _ = ndimage.label(circ_free)
        circ_reach = (lab == lab[sr, sc]) & (lab[sr, sc] > 0)
        rc = reached_goals(circ_reach)
        # se2 model
        hk = [k * 2 * math.pi / NH for k in range(NH)]
        free_h = [~collides(blocked, kernel(design, [th], res)) for th in hk]
        rot_free = [~collides(blocked, kernel(design, np.linspace(hk[k], hk[k] + 2 * math.pi / NH, 7), res)) for k in range(NH)]
        labs = [line_labels(free_h[k], steps[k]) for k in range(NH)]
        reach = [np.zeros((H, W), bool) for _ in range(NH)]
        start_h = 0
        if not free_h[start_h][sr, sc]:
            start_h = next((k for k in range(NH) if free_h[k][sr, sc]), None)
        if start_h is None:
            print(f'{dname}: spawn pose collides at every heading')
            continue
        reach[start_h][sr, sc] = True
        for it in range(400):
            changed = False
            for k in range(NH):
                g = grow(labs[k][0], labs[k][1], reach[k])
                if (g & ~reach[k]).any():
                    reach[k] |= g; changed = True
            for k in range(NH):
                k2 = (k + 1) % NH
                a = reach[k] & rot_free[k] & ~reach[k2]
                b = reach[k2] & rot_free[k] & ~reach[k]
                if a.any():
                    reach[k2] |= a; changed = True
                if b.any():
                    reach[k] |= b; changed = True
            if not changed:
                break
        se2_reach = np.logical_or.reduce(reach)
        rs = reached_goals(se2_reach)
        if isinstance(design, str):
            radius = float(design.split(':')[1]); dims = f'round d{2 * radius:.2f}'
        else:
            xs = [v for x0, x1, _, _ in design for v in (x0, x1)]; ys = [v for _, _, y0, y1 in design for v in (y0, y1)]
            radius = max(math.hypot(max(abs(x0), abs(x1)), max(abs(y0), abs(y1))) for x0, x1, y0, y1 in design)
            dims = f'{max(xs) - min(xs):.2f} x {max(ys) - min(ys):.2f}'
        results[dname] = (rc, rs)
        print(f'{dname}: {dims} m, swept radius {radius:.2f} m | circle model {len(rc)}/{len(goals)} goals | '
              f'se2 model {len(rs)}/{len(goals)} goals ({it + 1} iterations)')
    base = results.get(next(iter(DESIGNS)))
    allg = {g[0]: g[1] for g in goals}
    for dname, (rc, rs) in results.items():
        lost = sorted(set(allg) - rs)
        print(f'  {dname.split(" ")[0]} se2 unreachable: ' + ', '.join(f'{g} {allg[g]}' for g in lost))


if __name__ == '__main__':
    main()
