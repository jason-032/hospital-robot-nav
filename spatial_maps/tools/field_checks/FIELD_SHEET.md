# Field sheet: Go2 localisation, 19 to 23 October 2026

Based on the lab's description of the robot's software on 13 October 2026. None
of this has been run on the robot. Fill in the blanks on the day.

## The set-up this sheet assumes

- 3D LiDAR on top of the body, about 0.55 m above the floor, flattened by its
  driver into a 2D scan on `/ouster/scan` at 10 Hz.
- Odometry from the robot's own leg and IMU estimate (`/odom`), fused with IMU
  heading into `/odometry/filtered`. No laser odometry.
- AMCL matches the 2D scan to the map and publishes `map -> odom`. A localisation
  gate holds the last good estimate when the scan disagrees with the map.
- Frames `map -> odom -> base_footprint -> base_link -> os_sensor`.
- The map is made from the building model, not from a scan, at 0.05 m per cell.
- ROS 2 Humble on the robot. The start pose is set by hand at a known spot.

## What the lab's own numbers say

| Observation | What it points to |
| --- | --- |
| Hand-set pose differed from the best scan fit by about 0.4 m and 20 deg | Within the preflight's search (1.0 m, 30 deg). Heading is the weak part of hand placement. |
| Estimate drifted 1.2 m and 30 deg after a 1.5 m walk | Too large and too fast for odometry drift alone. Either AMCL is being pulled by things the map lacks, or heading is wrong in the odometry. Tests B and C separate the two. |
| One run crept 4.5 m into a wall | Same pattern as hospital 1F in simulation, where AMCL diverged 13 m in 25 s because the map and the world disagreed. |

A building-model map has walls and columns only. A lab at 0.55 m height is mostly
desks, chairs, shelves and people. Expect a low share of scan hits on the map
inside the lab and a much higher one in a corridor. That share is the first
number to write down.

## A. Before the robot moves (10 minutes)

On the robot's computer:

```bash
python3 localisation_preflight.py --scan-topic /ouster/scan --lifecycle-node <name of the lifecycle manager>
```

If it fails on Humble, run it on the operator PC (same ROS domain) and ignore the
transform ages unless the two clocks are synchronised.

| Place | Hits on the map (%) | Best fit dx, dy, yaw | Ambiguous? | Warnings |
| --- | --- | --- | --- | --- |
| Charging bay, hand-set pose | | | | |
| Charging bay, after applying the printed pose | | | | |
| Corridor, marked spot | | | | |
| Corridor, after applying the printed pose | | | | |

1. `[map]` should say 1 message.
2. `[tf]` should show `map -> odom` on `/tf` only, not on `/tf_static` as well.
3. Apply the `/initialpose` command the tool prints, run again, and repeat until
   the correction is zero. Stop if it says AMBIGUOUS.
4. Height of the 2D scan: find out which part of the 3D cloud the driver flattens.
   In RViz, with the map shown, check whether the scan draws desks and chair legs
   or only walls. Height band used: ________

Decision: if the lab scores under about 60% even after correction and the
corridor scores over 90%, run the test in the corridor and start from there.

## B. Odometry on its own (15 minutes, AMCL not needed)

Tape a 5 m straight line and a start mark. Record `/odom` and
`/odometry/filtered`.

| Test | Tape measure | `/odom` | `/odometry/filtered` |
| --- | --- | --- | --- |
| Stand still 60 s: change in heading | 0 deg | | |
| Walk 5 m straight: distance | 5.00 m | | |
| Walk 5 m straight: sideways error at the end | | | |
| Turn 360 deg on the spot: heading change | 360 deg | | |
| Turn 360 deg on the spot: position change | 0 m | | |

- Heading that changes while standing still is IMU drift.
- A 360 deg turn that reads much more or less than 360 is a heading scale or
  fusion fault, and would explain a 30 deg error after a short walk by itself.
- These errors, as a fraction of distance and angle, are what AMCL's `alpha1` to
  `alpha5` should be set from. Hospital 1F used 0.02 with near-perfect simulated
  odometry; leg odometry will need more.

## C. AMCL while walking (15 minutes)

From a corrected pose in the corridor, walk 1.5 m by hand control, stop, and run
the preflight. Repeat in the lab.

| Place | Hits before (%) | Hits after (%) | Fit after: dx, dy, yaw | `/localization_ok` | Tape-measured position error |
| --- | --- | --- | --- | --- | --- |
| Corridor | | | | | |
| Lab | | | | | |

- Good in the corridor, bad in the lab, with test B clean: the map is missing
  what the LiDAR sees. Fix the scan height or the test area, not AMCL.
- Bad in both, with test B clean: AMCL settings (`alpha` too low for leg
  odometry, too few beams, wrong laser range).
- Test B not clean: fix odometry first. Nothing downstream can be trusted.

## What to record

Two minutes standing on the marked spot, then tests B and C: `/tf`, `/tf_static`,
`/map`, `/ouster/scan`, `/odom`, `/odometry/filtered`, `/amcl_pose`, the three
localisation health topics, and the IMU. The preflight runs on a recording too:
play the bag and run the tool at the same time.

Bring: tape measure, floor tape, a way to mark heading (a long straight edge).
