# Field checks: localisation before a real-robot run

Written for the SCS Lab Go2 field test of 19 to 23 October 2026, where the open
problem is localisation: the robot had to be placed by hand to line up with the
map. `localisation_preflight.py` reads a running Nav2 + AMCL system and reports
the faults that broke localisation on hospital 1F in simulation. It is read-only:
it never publishes, sets a pose or changes a parameter.

```bash
python3 localisation_preflight.py                      # real robot
python3 localisation_preflight.py --sim-time           # Gazebo
python3 localisation_preflight.py --scan-topic /scan --base-frame base_link --lifecycle-node lifecycle_manager_localization
```

One file, no package build needed. Run it on the robot, or on a machine whose
clock is synchronised with the robot's, since transform ages compare message
stamps with the local clock. Written and tested on Jazzy; nothing Jazzy-specific
is used, but Humble is untested. Needs rclpy, tf2_ros, NumPy and SciPy. Exit code
1 if anything is flagged.

## What it does not do

- **It does not find a pose from scratch.** It refines a believed pose that is
  already within 1.0 m and 10 deg. If the robot is placed further off than that,
  or with no pose at all, it only says the fit is poor.
- **It needs a 2D `LaserScan`** and a transform from the map frame to the scan
  frame. A 3D LiDAR needs a levelled slice published as a `LaserScan` first, cut
  at a height where the map has walls. Only yaw is used, so a tilted sensor
  gives a wrong answer.
- **The `[amcl]` section only applies if AMCL is the localiser.** With another
  localiser it prints that the node did not answer; the map, transform and scan
  checks still apply.
- **A plain corridor or one of several alike rooms fits in more than one place.**
  The `[scan] fits within 3 points` line reports this as AMBIGUOUS, and then no
  corrected pose is offered.

## What it was tested on

Hospital 1F in Gazebo, 13 October 2026, robot stationary at spawn:

| Case | Scan hits on the map | Reported correction | Warnings |
| --- | --- | --- | --- |
| As launched | 99.4% | none | 0 |
| Initial pose pushed by +0.40 m, -0.30 m, +5 deg | 32.9% | -0.35 m, +0.25 m, -5 deg | 1 |

A wider test on the CBNU scan site in Gazebo (`sim_matrix.py`, 13 October 2026,
`~/sim_trials/2026-10-13/matrix_1.result`): 5 poses, each as it is and with three
known errors (+0.40, -0.30 m, +5 deg; -0.30, +0.20 m, -8 deg; +0.70 m along x).

| | Result |
| --- | --- |
| Correct poses that raised a warning | 0 of 5 |
| Wrong poses flagged | 15 of 15 |
| Corrected base pose within 0.10 m and 2 deg of truth | 15 of 15 (worst 0.05 m, 0.1 deg) |
| Reported as ambiguous | 0 of 20 |

This is the easiest case there is: the simulated world is built from the map,
so a correct pose scores 99 to 100%, the robot is standing still, and the errors
are inside the search range. A real building with furniture, people and glass
will score lower and may be ambiguous. Treat these numbers as "the tool does
what it says", not as what to expect on the Go2.

The correction search steps 0.05 m and 1 deg over +-1.0 m and +-10 deg, so it
cannot find an error larger than that and is no finer than one step.
Among fits within one point of the best it reports the smallest move, so it
tends to understate: here by 0.05 m on each axis. The corrected base pose it
prints (as an `/initialpose` command, never sent by the tool) is therefore a
step towards the right pose, not the exact one.
**It has not been run on the Go2 or on any real robot.** Topic, frame and node
names on the Go2 stack may differ; every one is a command-line option.

## Checklist

What carries over from 1F is the checks, not the numbers. The 1F values came from
a wheeled robot with near-perfect simulated odometry.

1. **Publish the map once.** On 1F the latched map was republished at 1 Hz. AMCL
   reinitialised on every copy, `map -> odom` went stale and every goal aborted.
   The `[map]` line should say 1 message.
2. **One owner of `map -> odom`.** A static publisher alongside AMCL fights it; a
   static publisher alone never corrects drift (0.6 m sideways over 35 m on 1F).
   The `[tf]` lines show both sources.
3. **Initial pose from the map, then checked by the scan.** Mark the start point
   and heading on the floor from map coordinates, set `initial_pose` to match,
   then run the tool before the first goal. `[scan] best fit` gives the remaining
   error, and the tool prints the corrected base pose as a command. Because it
   understates, apply it and run again; repeat until the correction is zero.
4. **How much of what the LiDAR sees is in the map.** This is the 1F failure most
   likely to recur. Where the map and the world disagreed (obstacles mapped with
   no physical geometry), AMCL with default noise diverged 13 m in 25 s. A BIM map
   of a room full of desks is the same mismatch the other way round. `[scan]`
   gives the share of hits on the map: 99% on 1F. A low share that no correction
   improves means clutter, and is the measured reason to test in the corridor.
   Check also that the scan is cut at a height where the map has walls.
5. **Motion noise must match the odometry.** 1F held at `alpha1..5 = 0.02` because
   odometry tracked truth to 0.1%. The Go2 has no wheel odometry, so measure
   first: drive 5 m straight and turn 360 deg on a marked floor and compare with
   `/odom`. Poor odometry needs larger alphas and a well matched map; good
   odometry lets AMCL coast through clutter. A robot that strafes needs the omni
   motion model.
6. **Sensor settings are the real sensor's.** `laser_max_range`, `laser_min_range`
   and enough beams (`max_beams` 60 was too few on 1F; 180 held).
7. **Large maps need a longer `bond_timeout`.** The 8 M cell 1F map blocked AMCL
   past the 4 s default and the lifecycle manager shut Nav2 down. 30 s held.
8. **Base frame at the centre of rotation, honest footprint.** On 1F a
   `base_footprint` 0.15 m ahead of the axle shifted the believed pose by up to
   0.30 m per turn, and a footprint smaller than the body clipped corners.
9. **Do not judge by Nav2 SUCCEEDED.** 1F reported successes for weeks while the
   robot could not physically turn. Measure the stopping point from a floor mark.
