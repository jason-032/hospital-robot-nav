# CLAUDE.md

Context for Claude Code working in this repository. Read before changing anything.

## What this is

A multi-floor autonomous robot navigation system for a 15-floor hospital, built entirely from BIM data. ROS2 Jazzy on Ubuntu 24.04, Gazebo Harmonic (Gz Sim 8), Nav2. Author: Jason Mwinsegre Dassah. Research direction proposed by Prof. Min-Koo Kim, Smart Construction and Systems Lab, Chungbuk National University.

Fifteen floors (B1F through 14F), each with its own occupancy map, semantic POI layer, topological graph, and Gazebo world. One parameterised entry point launches any of them.

**Current state:** 2F validated at 83% navigation success. 1F at 38%, limited by a known coordinate misalignment (see Open problems).

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
Nav2 — NavFn A*, Regulated Pure Pursuit @ 0.26 m/s, BT navigator, lifecycle manager
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

`spatial_maps_1f.launch.py` runs the full 1F simulation but omits the AMCL node.

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

## Decisions that look wrong but are deliberate

Do not "fix" these without reading why.

**`AMCL tf_broadcast: false`.** AMCL runs and subscribes to `/scan`, but a static `map -> odom` transform published at the spawn point is the primary source. This is because the 1F BIM wall mesh sits ~0.9 m from the PGM origin, so AMCL scan-matching produces incorrect corrections. Re-enabling this depends on fixing the alignment first.

**1F uses Bullet Featherstone, not DART.** The default DART engine crashes on the 17,051-face BIM wall mesh.

**The BIM wall mesh has no collision geometry.** It is visual only. Its coordinate offset would place physical obstacles inside Nav2 inflation zones and trap the robot. An invisible perimeter fence of four collision-only boxes, 1 m outside each PGM edge, stops the robot escaping the footprint without touching the costmap.

**Nav2 `bond_timeout` is 30s, not the default 4s.** The 8.26 M cell (2075 x 3982 px) map blocks the executor longer than the default heartbeat window, which crash-loops AMCL.

**`map_publisher_node` repeat rate is 0.001 Hz with a 1s startup timer.** The map must be on the wire before the lifecycle manager activates nodes at roughly t=13s, but republishing an 8 M cell map frequently is wasteful. TransientLocal QoS covers late subscribers.

**Goal projection is not a simple centroid.** BIM room centroids routinely fall inside walls. `poi_nav_node.py` does reachability BFS from spawn, a scipy `distance_transform_edt` for clearance, then two-pass BFS projection (free-space only, then wall-crossing as fallback), with a 0.65 m clearance threshold (0.55 m inflation + 0.10 m buffer).

## Open problems

**1. BIM-to-PGM coordinate misalignment (highest priority).** The 1F wall mesh is exported ~0.9 m off the PGM origin with a scale discrepancy that no single rigid transform corrects. Root fix is re-exporting the wall geometry from the BIM in the same frame used to generate the PGM. This blocks physical collision geometry, correct AMCL localisation, and a meaningful 1F pass rate. Everything else is downstream of this.

**2. Odometry drift on 1F.** With no interior collision and AMCL TF broadcast off, drift places the robot's computed pose inside an occupied cell, and NavFn rejects every goal. `sweep_test.py` cascade recovery mitigates it. Caused by item 1.

**3. Topological module not integrated.** `network_navigation_node.py` publishes `cmd_vel` directly, which conflicts with Nav2's controller server. Needs a waypoint-handoff interface and cmd_vel arbitration.

**4. `models/pgm_walls_1f/` is not installed.** `models/` is absent from the `install(DIRECTORY ...)` block in `CMakeLists.txt`. Reference by full source path or extend CMakeLists.

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

Next phases, in order: fix the coordinate alignment, re-enable full AMCL, implement multi-floor transition via elevators, add dynamic costmap layers from the LiDAR, extend to multi-robot, validate on a second IFC building.

Longer term this stack is intended to move from simulation onto a physical quadruped operating in a real building. That will require LiDAR-inertial odometry rather than the current static-TF approach, since legged platforms have no wheel odometry and leg odometry drifts badly. Treat any simulation-only shortcut in this repository as temporary and flag it rather than building on it.
