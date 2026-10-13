#!/usr/bin/env python3
"""Test localisation_preflight.py against known pose errors in a running simulation.

For each test pose the robot is moved there in Gazebo, AMCL is given that pose
plus a known error, and the preflight is run. The table shows the error given,
what the tool reported, and how far its corrected pose is from the truth.
Simulation only: it teleports the robot and publishes /initialpose.

Usage (simulation running, ROS environment sourced):
  sim_matrix.py <gz world name> <out dir> "x,y,yaw_deg" ["x,y,yaw_deg" ...]
"""
import math
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
# (dx m, dy m, dyaw deg) added to the true pose before it is given to AMCL.
ERRORS = [(0.0, 0.0, 0.0), (0.40, -0.30, 5.0), (-0.30, 0.20, -8.0), (0.70, 0.0, 0.0)]


def sh(cmd, timeout=60):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout).stdout


def main():
    world, out = sys.argv[1], sys.argv[2]
    poses = [tuple(float(v) for v in a.split(',')) for a in sys.argv[3:]]
    os.makedirs(out, exist_ok=True)
    rows = []
    for pi, (x, y, yaw_deg) in enumerate(poses):
        yaw = math.radians(yaw_deg)
        sh(f"gz service -s /world/{world}/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean "
           f"--timeout 3000 --req 'name: \"my_robot\", position: {{x: {x}, y: {y}, z: 0.1}}, "
           f"orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}'")
        time.sleep(3)
        for ei, (ex, ey, eyaw) in enumerate(ERRORS):
            gy = yaw + math.radians(eyaw)
            sh("ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
               f"\"{{header: {{frame_id: map}}, pose: {{pose: {{position: {{x: {x + ex}, y: {y + ey}}}, "
               f"orientation: {{z: {math.sin(gy / 2)}, w: {math.cos(gy / 2)}}}}}, covariance: "
               "[0.0025,0,0,0,0,0, 0,0.0025,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0003]}}\"")
            time.sleep(4)
            p = subprocess.run(['python3', os.path.join(HERE, 'localisation_preflight.py'),
                                '--sim-time', '--duration', '8'], capture_output=True, text=True)
            open(os.path.join(out, f'matrix_p{pi}_e{ei}.txt'), 'w').write(p.stdout + p.stderr)
            fit = re.search(r'best fit: move the sensor by dx ([-+.\d]+) m, dy ([-+.\d]+) m, yaw ([-+\d]+) deg', p.stdout)
            new = re.search(r'to \(([-.\d]+), ([-.\d]+), ([-.\d]+) deg\)\. Not sent', p.stdout)
            hits = re.search(r'\[scan\] ([\d.]+)% of hits', p.stdout)
            amb = 'AMBIGUOUS' in p.stdout
            warns = [l[6:60] for l in p.stdout.splitlines() if l.startswith('WARN')]
            if new:
                rx, ry = float(new.group(1)) - x, float(new.group(2)) - y
                ryaw = (float(new.group(3)) - yaw_deg + 180) % 360 - 180
            else:       # no correction offered: the believed pose stands
                rx, ry, ryaw = ex, ey, eyaw
            rows.append((pi, (x, y, yaw_deg), (ex, ey, eyaw), hits.group(1) if hits else '-',
                         fit.groups() if fit else '-', amb, math.hypot(rx, ry), abs(ryaw), warns))
    print('pose | true pose | error given | hits % | sensor fit | ambiguous | residual m | residual deg | warnings')
    for r in rows:
        print(' | '.join(str(v) if not isinstance(v, float) else f'{v:.2f}' for v in r))
    given = [r for r in rows if r[2] != (0.0, 0.0, 0.0)]
    healthy = [r for r in rows if r[2] == (0.0, 0.0, 0.0)]
    print(f'healthy poses with a warning: {sum(bool(r[8]) for r in healthy)}/{len(healthy)}')
    print(f'wrong poses flagged (warning or ambiguous): {sum(bool(r[8]) or r[5] for r in given)}/{len(given)}')
    print(f'wrong poses with a corrected pose within 0.10 m and 2 deg: '
          f'{sum(r[6] <= 0.10 and r[7] <= 2 and not r[5] for r in given)}/{len(given)}')


if __name__ == '__main__':
    main()
