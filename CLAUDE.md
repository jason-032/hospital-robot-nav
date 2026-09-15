# CLAUDE.md

Context for Claude Code working in this repository. Read before changing anything.

## What this is

A multi-floor autonomous robot navigation system for a 15-floor hospital, built entirely from BIM data. ROS2 Jazzy on Ubuntu 24.04, Gazebo Harmonic (Gz Sim 8), Nav2. Author: Jason Mwinsegre Dassah. Research direction proposed by Prof. Min-Koo Kim, Smart Construction and Systems Lab, Chungbuk National University.

Fifteen floors (B1F through 14F), each with its own occupancy map, semantic POI layer, topological graph, and Gazebo world. One parameterised entry point launches any of them.

**Current state:** 2F recorded 83% sweep success, not yet checked against ground truth. **All 1F sweep figures (38%, and 73 of 74 on 15 September 2026) are withdrawn:** ground-truth logging showed the robot could not physically turn under Bullet Featherstone, so they measured belief only. The contact-surface fix for that is in place and validated by motion tests. On branch `feat/1f-mesh-collision` the aligned mesh carries collision in place of the 1108 PGM boxes. The first logged sweep after the fix exposed an odometry frame offset, an undersized costmap footprint and uncorrected localization (see Open problems 2). Treat `sweep_test.py` SUCCESS as belief until checked with `tools/sim_checks/gt_analyse.py`.

## Architecture

```
IFC BIM model (upstream pipeline, not in this repo)
  ├── semantic.json        per-floor IfcSpace entities, centroids, robot_response
  ├── maps/*.pgm + *.yaml  occupancy grids at 0.02 m/px
  └── network/*.json       topological graph, 5,542 nodes / 3,259 edges
            ↓
Gazebo Harmonic — 15 SDF 1.10 worlds, robot spawns at (21.0, 38.0, 0.1) map frame
            ↓  ros_gz_bridge (per-floor YAML)
my_robot_description — diff-drive base, 2-DOF arm, RGB camera, 360° GPU LiDAR
            ↓
Nav2 — NavFn A*, Regulated Pure Pursuit @ 0.45 m/s, BT navigator, lifecycle manager
            ↑
POI layer — poi_publisher (RViz markers), poi_click_node, poi_nav_node (goal projection)
```

## Packages

| Package | Contains |
| --- | --- |
| `my_robot_bringup` | Standalone robot launch, Gz bridge config, test world |
| `my_robot_description` | URDF/Xacro robot model, Gazebo plugin config |
| `spatial_maps` | The multi-floor hospital system. Almost all work happens here. |

## Entry points

**Use this one:**
```bash
ros2 launch spatial_maps spatial_maps.launch.py floor:=1F
```
Valid floors: `B1F`, `1F`, `2F` through `14F`. Wait 15 to 20 seconds for `[lifecycle_manager_nav] Managed nodes are active`.

**Do not assume the per-floor launch files do what their names suggest.** `floor_1f.launch.py` and its thirteen siblings start only a `map_server` plus lifecycle manager. They do not start Gazebo, the robot, Nav2, or any bridge. They are map-server stubs.

`spatial_maps_1f.launch.py` runs the full 1F simulation, including AMCL (with `tf_broadcast: false`). It is the launch file `sweep_test.py` runs have used; `headless:=true` runs Gazebo server-only.

## Key nodes

| Script | Role |
| --- | --- |
| `map_publisher_node.py` | PGM to `/map`, TransientLocal QoS, 1s startup timer |
| `poi_nav_node.py` | Goal projection and Nav2 action client. The hardest logic in the repo. |
| `poi_click_node.py` | RViz Publish Point to nearest room to `/goal_poi` |
| `poi_publisher_node.py` | BIM semantics to RViz MarkerArray, colour-coded by `robot_response` |
| `network_navigation_node.py` | Dijkstra over the BIM graph. **Standalone prototype, not launched.** |
| `odom_tf_republisher.py` | Retimestamped `odom -> base_footprint`, monotonic guard |
| `joint_state_relay.py` | Retimestamped Gz to ROS joint states |
| `sweep_test.py` | Automated per-floor POI sweep with cascade recovery, CSV output |
| `pgm_to_sdf_walls.py` | Utility: PGM to Gazebo SDF collision boxes |
| `glb_to_wall_mesh.py` | Utility: pipeline floor GLB to Gazebo wall OBJ in the map frame, with an alignment check against the PGM |

## Decisions that look wrong but are deliberate

Do not "fix" these without reading why.

**`AMCL tf_broadcast: false`.** AMCL runs and subscribes to `/scan`, but a static `map -> odom` transform published at the spawn point is the primary source. This was originally forced by the misaligned 1F wall mesh, which made AMCL scan-matching produce incorrect corrections. The mesh is now aligned (LiDAR hits within 0.06 m of occupied cells rose from 53% to 99%), so the blocker is gone, but re-enabling has not yet been validated with a sweep. Do not flip it without running one.

**1F uses Bullet Featherstone, not DART.** DART crashed when the earlier 17,051-face wall mesh carried collision geometry, and on 15 September 2026 it also segfaulted on the 7,122-face mesh with both its ODE and Bullet collision detectors (stack traces in `OdeMesh::fillArrays` and `BulletCollisionDetector::createBulletCollisionShape`). DART runs 1F fine without mesh collision (`tools/sim_checks/worlds/1f_nocolldart.world`).

**Robot contact surfaces carry Bullet-specific settings, and the 1F ground plane has torsional friction 0.** In `my_robot_description/urdf/mobile_base_gazebo.xacro` the caster is a preserved fixed-joint link with `<ode><mu>` and `<bullet><friction>` both 0.1, and wheels and caster have `<torsional><coefficient>0`. Bullet ignores `<ode><mu>`, sets friction once per link (the lumped caster inherited the body's 1.0), and applies torsional coefficient as spinning friction combined from both surfaces. With the defaults the robot turned 0.000 of commanded and drove 0.58 of commanded distance. Removing any one of these settings, or the ground plane's torsional 0, brings the fault back (`tools/sim_checks/bullet_friction_test.py`, results in `~/sim_trials/2026-09-15/friction_test*.result`). DART behaves identically with or without them. Any new floor world run under Bullet needs the ground plane setting too.

**No colon-space inside URDF or xacro XML comments.** The launch files pass `robot_description` through a YAML parser, and `key: value`-shaped comment text makes the launch exit immediately.

**1F physical walls come from the BIM wall mesh, not the PGM boxes (under test).** The aligned mesh carries collision and `pgm_walls_1f` is no longer included. In a drive-into-wall test the mesh stopped the robot 0.316 m before the wall face (about half the 0.6 m chassis), confirmed by ground truth and LiDAR range. The same test in the boxes world showed the robot driving straight through PGM box sheets and on to x = 33.19 (see Open problems). Idle real-time factor was 1.00 with the mesh against 0.13 with the boxes (one measurement each so far). The earlier mesh could not carry collision because it contained IfcSpace room volumes and closed door panels, which trapped the robot. Trade-off: 55 PGM obstacle regions (84.9 m², 36% of occupied cells) contain no IfcWall or IfcColumn, so they are in the costmap but not physical. An invisible perimeter fence of four collision-only boxes, 1 m outside each PGM edge, stops the robot escaping the footprint without touching the costmap.

**The wall mesh contains only IfcWall and IfcColumn.** `glb_to_wall_mesh.py` deliberately leaves out IfcSpace (a full-height box per room) and IfcDoor (closed panels). Adding either back seals rooms that the carved PGM treats as reachable: with the old mesh, robot-reachable free space split into 68 regions instead of 14. Regenerate with the script rather than editing the OBJ, and never add an XY offset or pose: the GLB is already in the map frame.

**Nav2 `bond_timeout` is 30s, not the default 4s,** in both `spatial_maps.launch.py` and `spatial_maps_1f.launch.py`. The 8.26 M cell (2075 x 3982 px) map blocks the executor longer than the default heartbeat window, which crash-loops AMCL. The 1F launch file only gained the setting when mesh collision raised the real-time factor to about 1.0: with 4 s, AMCL crash-looped in 2 of 2 mesh-world runs (0 of 2 boxes-world runs of the same length) and took the Nav2 servers down with it.

**`map_publisher_node` repeat rate is 0.001 Hz with a 1s startup timer.** The map must be on the wire before the lifecycle manager activates nodes at roughly t=13s, but republishing an 8 M cell map frequently is wasteful. TransientLocal QoS covers late subscribers.

**Goal projection is not a simple centroid.** BIM room centroids routinely fall inside walls. `poi_nav_node.py` does reachability BFS from spawn, a scipy `distance_transform_edt` for clearance, then two-pass BFS projection (free-space only, then wall-crossing as fallback), with a 0.65 m clearance threshold (0.55 m inflation + 0.10 m buffer).

## Open problems

**1. 1F wall mesh alignment: mesh fixed, downstream steps pending (highest priority).** The reported ~0.9 m offset and "scale discrepancy" were not in the BIM. The pipeline's `floor_1F.glb` already matches `1F.pgm` (100% of wall slice samples within 0.06 m). The old OBJ had been converted with +0.9107 m added to X (the PGM origin applied twice) and was 68% IfcSpace volumes, which no transform can fit to walls. The world pose of -1 m had cancelled most of the offset, so the real damage came from the room volumes and door panels: 30% of LiDAR hits landed more than 0.30 m from any occupied cell. `glb_to_wall_mesh.py` now regenerates the mesh with identity pose, and in simulation 99.3% of LiDAR hits fall within 0.06 m of occupied cells. Collision on the aligned mesh is now enabled on `feat/1f-mesh-collision` and under test (repeat short trials and two full sweeps per world). Still to do after that: AMCL `tf_broadcast: true`, then a full 1F sweep against the 38% baseline. Only 1F has a mesh; GLBs exist for B1F to 13F but not 14F.

**2. Believed pose vs true pose on 1F (highest priority).** The 0.58 odometry ratio and the inability to turn were a Bullet contact-surface problem, now fixed (see Decisions). The first ground-truth-logged sweep after the fix (`~/sim_trials/2026-09-15/p3_fix_mesh_sweep_1.*`) drove about 35 m correctly, then clipped a mapped wall corner near (10.4, 8.2) at 0.45 m/s, spun about 40° and later overturned. The mesh matches the PGM there within 0.10 m, so no hidden geometry was involved. Three open causes:
(a) **Frame offset.** DiffDrive odometry follows the wheel-axle midpoint, but `base_footprint` is 0.15 m ahead of it (wheel joints at `base_link` x = -0.15). A turn of Δ shifts the believed pose by 0.3·|sin(Δ/2)|, at most 0.30 m. Agreed fix: move `base_footprint` onto the axle.
(b) **Footprint.** The costmaps use `robot_radius: 0.27`, but the body reaches 0.36 m (0.52 m once the frame moves to the axle). The user wants an honest polygon footprint without losing reachable rooms, so design options are being evaluated.
(c) **AMCL** TF broadcast is still off, so nothing corrects residual error. Enable it after (a) and (b) are validated, in a separate logged sweep.
Also: the arm makes the robot top-heavy, so collisions can overturn it. `sweep_test.py` cascade recovery still mitigates start poses inside occupied cells.

**3. Topological module not integrated.** `network_navigation_node.py` publishes `cmd_vel` directly, which conflicts with Nav2's controller server. Needs a waypoint-handoff interface and cmd_vel arbitration.

**4. The PGM box model does not stop the robot.** In a drive-into-wall test in the boxes world, the robot passed through the `pgm_walls_1f` box sheets at x = 24.92 and 25.16 and continued to x = 33.19 without stalling. Unverified hypothesis: the model is 1108 unjointed links in one static model, which Bullet Featherstone may not build as colliders; the mesh is a single link and does collide. This also means the July curve-wedging analysis, which attributed jams to solid PGM walls, needs re-examination; it was also run while the robot could not turn under Bullet (Open problem 2), so it describes believed motion only.

**5. `models/pgm_walls_1f/` is not installed.** `models/` is absent from the `install(DIRECTORY ...)` block in `CMakeLists.txt`. Reference by full source path or extend CMakeLists.

## External dependency

`semantic.json` comes from the upstream Indoor Spatial Map Generation Pipeline and is **not in this repository**. Default path:

```
/home/jason/Downloads/OneDrive_1_4-10-2026/entity/semantic.json
```

Override with the `semantic_json` launch parameter. The 83% and 38% pass rates depend on this dataset and cannot be reproduced from repository contents alone.

This path is fragile. Moving it to a configured location is a worthwhile small task.

## Conventions

- Python 3.12. NumPy, SciPy, PyYAML, Pillow.
- Build with `colcon build --symlink-install` from `~/ros2_ws`, then source `install/setup.bash`.
- New scripts in `spatial_maps/scripts/` must be added to `CMakeLists.txt` to be installed.
- Prose in documentation and commit messages: British spelling, no em-dashes.
- Do not overstate results. The README's Known Limitations section is deliberately rigorous and new limitations should be recorded there rather than smoothed over.

## Direction

Next phases, in order: fix the 1F odometry frame offset and costmap footprint, remeasure 1F with ground-truth logging, re-enable full AMCL, check 2F against ground truth, implement multi-floor transition via elevators, add dynamic costmap layers from the LiDAR, extend to multi-robot, validate on a second IFC building.

Longer term this stack is intended to move from simulation onto a physical quadruped operating in a real building. That will require LiDAR-inertial odometry rather than the current static-TF approach, since legged platforms have no wheel odometry and leg odometry drifts badly. Treat any simulation-only shortcut in this repository as temporary and flag it rather than building on it.
