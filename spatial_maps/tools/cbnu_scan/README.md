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

**2 of 11 rooms reached, plus a drive back to the start point, in two runs**
(`cbnu_planpad_1`, `cbnu_default_1`), each confirmed by ground truth. The rooms
are ClassRoom_5 and HallWay_4. The sweep also reports HallWay_3 as reached, but
its goal is the spawn cell itself: the robot drove about 9 m back to where it
started, which is real motion and no new space. True error 0.20 to 0.32 m,
believed pose within 0.06 m of truth, travel ratio 1.00 over 56 m, no true pose
inside an occupied or unknown cell. The other eight goals were refused by the
planner ("no valid path found"); none failed while driving.

Read the figure with these limits in mind:

- **It is about this robot on this map.** The robot needs a 0.75 m gap to drive
  and the planner asks for 0.85 m; the Go2 is narrower.
- **Three goals are not in the room they are named after.** The centres of
  ClassRoom_1, ClassRoom_3 and Study_room_1 fall on occupied cells, and the sweep's
  0.85 m clearance projection moved them 3.9 to 5.5 m, into the neighbouring
  corridor. A success on one of those would be a corridor point.
- **The result sits on a knife edge.** An offline check that treats the robot as
  a circle reaches 6 of 11 goals at radii up to 0.425 m and 3 at 0.45 m. The
  three lost (ClassRoom_3, HallWay_2, Study_room_1) lie beyond one gap of about
  0.8 m near x = -34.8, y = -20.9.
- **The obstacles that decided it look like scan speckle, not building.** That gap
  is a free-standing diagonal streak of occupied cells in a corridor about 2 m
  wide, and the post in the table below is a single cell. The map has 50
  single-cell occupied blobs. A cleaned map would probably give a different
  number; this has not been tried.
- **At this robot's width,** ClassRoom_1, ClassRoom_2 and HallWay_1 can only be
  reached across cells the scan never covered. A scanned passage about 0.45 m
  wide exists.

## What went wrong on the way, in order

| Run | Settings | Outcome |
| --- | --- | --- |
| `cbnu_sweep_1` | 1F settings unchanged | Nav2 reported 2 successes, both false. True pose froze on goal 1 at (-65.67, -7.74) and ended 65 m from the believed pose. |
| `cbnu_pad010_1` | footprint padding 0.10 m | Robot pitched over backwards at the same place and ended upside down, body 0.3 m up. 0 of 11. |
| `cbnu_repeat_1` | 1F settings, goal 1 only | Passed that place 5 cm further out and kept going, but 210 of 409 true poses were in unscanned cells and localisation error grew to 2.2 m. |
| `cbnu_wheels_unknown_1`, `cbnu_visible_1` | wheel-inclusive footprint, unknown space blocked | 1 of 11, and that one (ClassRoom_3) is a corridor point, see above. Robot then stopped in a 0.775 m gap and every later plan failed with "Start occupied". |
| `cbnu_planpad_1`, `cbnu_default_1` | plus planner padding 0.10 m | 2 rooms plus the return to start, both times; robot never stuck. One "Start occupied" in the second run, during a replan on the way to HallWay_4; it recovered and reached the goal. |

The three settings are in `config/nav2_params_cbnu_scan.yaml`, which the CBNU
launch file loads by default:

1. **Footprint includes the wheel collision shapes.** They are spheres of radius
   0.10 m at y +-0.225, reaching y +-0.325, while the 1F footprint stops at
   y +-0.25. What the data show: the robot pitched over when the post was 0.334 m
   and 0.350 m to its side, and passed cleanly at 0.387 m, and nothing but a
   wheel sphere reaches that far out. So a wheel touching the post is well
   supported. **That the narrow footprint is the cause is not established:**
   the second pitch-over happened with padding 0.10 m, a half-width of 0.35 m,
   wider than the sphere. Costmap cell rounding, a 0.05 m localisation error, the
   collision margin on a zero-thickness mesh, and wheel joints with no torque
   limit are all candidates. The robot never passed that post again once unknown
   space was blocked, so the wider footprint is untested against it. A controlled
   drive-past test is needed. **The 1F configuration still has the narrower
   footprint.**
2. **Unknown cells are not free.** The 1F map has no unknown cells, so the 1F
   global costmap never needed `track_unknown_space`. Separately, the bundle's
   map YAML has `free_thresh: 0.25`, which loads unknown pixels (occupancy 0.196)
   as free; `build_site.py` writes 0.196. This blocks unknown cells for the
   planner only. The local costmap, which the controller checks, still treats
   them as free; no true pose was in an unknown cell in the last four runs.
3. **The planner pads the footprint by 0.10 m, the controller by 0.05 m,** so a
   path is never planned through a gap the robot fits with no tracking margin.

## Not done

- Each configuration was run once or twice, the final one twice.
- Nothing here has been checked against the real building or a real robot.
- The wall mesh is zero-thickness quads on cell edges, 1 m tall. Clutter, glass
  and anything outside the 0.08 to 0.60 m height band are not in the world.
- The doorway speed mask is blank (no limit anywhere).
