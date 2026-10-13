# CBNU scan map in the simulator

Drives the hospital robot on a Nav2 map that came from a laser scan instead of a
BIM model: `cbnu_scan_map.pgm` from the cbnu-scan2bim TLS study (11 spaces of the
CBNU engineering building, 0.05 m cells). The Gazebo world is the outline of the
map's own occupied cells, so a run tests whether the stack can drive the map. It
says nothing about whether the map matches the building.

```bash
python3 build_site.py <unzipped bundle dir>     # writes maps, mesh, world, config
bash run_trial.sh <label>                       # launch, checks, 11-room sweep, ground truth
HEADLESS=false RVIZ=true bash run_trial.sh <label>    # same, with the Gazebo window and RViz
```

The bundle is release `cbnu-progress-2026-09-29` of the private claude-workspace
repository. Results land in `~/sim_trials/<date>/`.

## Result, 13 October 2026

**3 of 11 rooms reached, all three confirmed by ground truth, in two runs**
(`cbnu_planpad_1`, `cbnu_default_1`):
ClassRoom_5, HallWay_3 (which contains the spawn point) and HallWay_4, true error
0.20 to 0.32 m, believed pose within 0.06 m of truth, travel ratio 1.00 over 56 m,
no true pose inside an occupied cell. The other eight goals were refused by the
planner ("no valid path found"), none failed while driving.

This is a statement about this robot on this map, not about the map alone. The
robot needs a 0.75 m gap to drive and the planner asks for 0.85 m; the Go2 is
narrower. An offline clearance check that treats the robot as a 0.375 m circle
puts about 6 of 11 rooms in reach, so the planner margin costs about three rooms
(ClassRoom_3, HallWay_2 and Study_room_1 lie beyond one 0.775 m gap near
x = -34.8, y = -20.9). ClassRoom_1, ClassRoom_2 and HallWay_1 can only be reached
across cells the scan never covered.

## What went wrong on the way, in order

| Run | Settings | Outcome |
| --- | --- | --- |
| `cbnu_sweep_1` | 1F settings unchanged | Nav2 reported 2 successes, both false. True pose froze on goal 1 at (-65.67, -7.74) and ended 65 m from the believed pose. |
| `cbnu_pad010_1` | footprint padding 0.10 m | Robot flipped over at the same place (roll 180 deg, body 0.3 m up). 0 of 11. |
| `cbnu_repeat_1` | 1F settings, goal 1 only | Passed that place 5 cm further out and kept going, but 210 of 409 true poses were in unscanned cells and localisation error grew to 2.2 m. |
| `cbnu_wheels_unknown_1`, `cbnu_visible_1` | wheel-inclusive footprint, unknown space blocked | 1 of 11 (ClassRoom_3). Robot then stopped in a 0.775 m gap and every later plan failed with "Start occupied". |
| `cbnu_planpad_1`, `cbnu_default_1` | plus planner padding 0.10 m | 3 of 11 both times, robot never stuck. One "Start occupied" in the second run, during a replan on the way to HallWay_4; it recovered and reached the goal. |

The three settings are in `config/nav2_params_cbnu_scan.yaml`, which the CBNU
launch file loads by default:

1. **Footprint includes the wheel collision shapes.** They are spheres of radius
   0.10 m at y +-0.225, reaching y +-0.325, while the 1F footprint stops at
   y +-0.25. At the flip a single-cell post sat 0.325 m to the robot's side. This
   is geometry that fits, and the flip did not recur, but the robot never passed
   that post again once unknown space was blocked, so it is not a controlled
   test. **The 1F configuration still has the narrower footprint.**
2. **Unknown cells are not free.** The 1F map has no unknown cells, so the 1F
   global costmap never needed `track_unknown_space`. Separately, the bundle's
   map YAML has `free_thresh: 0.25`, which loads unknown pixels (occupancy 0.196)
   as free; `build_site.py` writes 0.196.
3. **The planner pads the footprint by 0.10 m, the controller by 0.05 m,** so a
   path is never planned through a gap the robot fits with no tracking margin.

## Not done

- Each configuration was run once or twice, the final one twice.
- Nothing here has been checked against the real building or a real robot.
- The wall mesh is zero-thickness quads on cell edges, 1 m tall. Clutter, glass
  and anything outside the 0.08 to 0.60 m height band are not in the world.
- The doorway speed mask is blank (no limit anywhere).
