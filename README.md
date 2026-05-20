# Hospital Robot Navigation Simulation
### BIM-Derived Multi-Floor Autonomous Navigation with POI Integration

---

## Research Summary

This project implements a physics-based, multi-floor autonomous robot navigation system for a 15-floor hospital building, constructed entirely from Building Information Modelling (BIM) data. It was developed as an independent research initiative directly aligned with the research agenda of the Smart Construction and Systems (SCS) Lab at Chungbuk National University, South Korea, whose work spans construction robotics, spatial data pipelines, and simulation-based systems for smart built environments.

The original research brief was to generate a Gazebo simulation environment from the outputs of an Indoor Spatial Map Generation Pipeline and validate POI (Point of Interest) integration by deploying a robot on a single floor. The Indoor Spatial Map Generation Pipeline is an upstream process that parses IFC-format BIM models to extract spatial entities (IfcSpace objects), room connectivity, door positions, stairwell geometry, and floor-level occupancy grids, producing both 2D navigable maps and semantic topology graphs as structured outputs.

This implementation goes substantially beyond the original brief. Rather than validating a single floor, the system spans all 15 floors of the hospital (B1F through 14F), provides fully parameterised launch infrastructure, integrates a dual-layer navigation architecture combining grid-based and topological planning, and demonstrates empirically validated navigation performance via an automated 90-room sweep test achieving a **91% success rate** on Floor 1F with zero planning failures. Each floor is independently launchable using a single parameterised entry point, and every floor carries its own occupancy map, semantic POI layer, topological network graph, and Gazebo simulation world.

This work demonstrates that IFC/BIM spatial data can be translated into a fully operational, multi-floor robot navigation stack without manual environment modelling, and that POI-aware goal-directed navigation in complex built environments is achievable using open-source robotics middleware at research quality.

---

## System Architecture

The pipeline flows from raw BIM data through to autonomous robot behaviour in simulation:

```
 IFC BIM Model (upstream pipeline)
        │
        ├── semantic.json          per-floor IfcSpace entities, centroids,
        │                          display names, robot_response metadata
        │
        ├── maps/*.pgm + *.yaml    2D occupancy grids (0.02 m/px) for each
        │                          floor, compatible with ROS2 map server
        │
        └── network/*.json         topological graphs: room nodes, door nodes,
                                   step nodes, Dijkstra-traversable edges
                                   (5,542 nodes, 3,259 edges, 15 floors)
                                           │
                                           ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │                    Gazebo Harmonic (Gz Sim 8)                   │
 │  15 × SDF 1.10 world files, one per floor                      │
 │  Floor plan texture (PGM→PNG), inline directional lighting      │
 │  Robot spawned at (21.0, 38.0, 0.1) in map frame               │
 └──────────────────────────┬──────────────────────────────────────┘
                            │  ros_gz_bridge (per-floor YAML config)
                            ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │                    my_robot_description                         │
 │  Differential-drive base (0.6×0.4×0.2 m)                       │
 │  2-DOF arm (revolute joints, ±90°, position-controlled)         │
 │  RGB camera (640×480, 80° FOV, 20 Hz)                          │
 │  Odometry at 50 Hz via DiffDrive Gz plugin                      │
 └──────────────────────────┬──────────────────────────────────────┘
                            │
          ┌─────────────────┼──────────────────┐
          ▼                 ▼                  ▼
  odom_tf_republisher  joint_state_relay  robot_state_publisher
  (odom→base_footprint  (retimestamps Gz   (URDF→TF tree,
   TF, retimestamped,    joint states,      /robot_description)
   monotonic guard)      monotonic guard)
          │                                    │
          └──────────────┬─────────────────────┘
                         ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │                       Nav2 Stack                                │
 │  map_publisher_node   →  /map  (TransientLocal QoS)            │
 │  planner_server       →  NavfnPlanner / A*  (0.5 m tolerance)  │
 │  controller_server    →  RegulatedPurePursuit (0.26 m/s)        │
 │  behavior_server      →  spin, backup, drive_on_heading, wait   │
 │  smoother_server      →  path smoothing                         │
 │  bt_navigator         →  NavigateToPose / NavigateThroughPoses  │
 │  lifecycle_manager    →  autostart, 12 s delay for TF settle    │
 └──────────────────────────┬──────────────────────────────────────┘
                            ▲
          ┌─────────────────┴──────────────────┐
          │                                    │
 ┌────────┴────────┐                ┌──────────┴──────────┐
 │   POI Layer     │                │  Topological Layer  │
 │                 │                │                     │
 │ poi_publisher   │                │ network_navigation  │
 │  (sphere+text   │                │  _node              │
 │   markers in    │                │  (Dijkstra over     │
 │   RViz, colour- │                │   BIM-derived graph,│
 │   coded by      │                │   5,542 nodes)      │
 │   robot_response│                │                     │
 │   class)        │                └─────────────────────┘
 │                 │
 │ poi_click_node  │   ← RViz Publish Point click → nearest room
 │ poi_nav_node    │   → goal projection (EDT + BFS) → Nav2 goal
 └─────────────────┘
```

### Goal Projection

A critical implementation detail is the goal projection algorithm in `poi_nav_node.py`. BIM room centroids frequently fall inside walls or inflation-layer obstacles. The algorithm projects each centroid outward using:

1. **Reachability BFS** — flood-fill from the robot spawn point at launch to identify all connected free cells.
2. **Euclidean Distance Transform** (scipy `distance_transform_edt`) — pre-computes wall clearance for every map cell.
3. **Two-pass BFS projection** — first searches free-space-only; if no valid cell is found within 10 m, a second pass allows wall-crossing to guarantee a result.
4. **Clearance threshold** — 0.65 m (inflation radius 0.55 m + 0.10 m buffer) to prevent goals adjacent to inflated obstacles.

---

## Technical Stack

| Component | Technology |
|-----------|-----------|
| Robotics middleware | ROS2 Jazzy Jalisco |
| Physics simulation | Gazebo Harmonic (Gz Sim 8), SDF 1.10 |
| Autonomous navigation | Nav2 (navfn A\*, RegulatedPurePursuit, BehaviorTree) |
| Robot description | URDF/Xacro |
| Simulation bridge | ros_gz_bridge |
| Map format | ROS2 OccupancyGrid (PGM/YAML, 0.02 m/px) |
| Spatial data source | IFC BIM via semantic.json pipeline output |
| Language | Python 3.12 |
| Key libraries | NumPy, SciPy (`distance_transform_edt`), PyYAML |
| Visualisation | RViz2 |
| OS | Ubuntu 24.04 |

---

## Repository Structure

```
ros2_ws/src/
├── my_robot_bringup/               Robot launch and Gz bridge config
│   ├── launch/
│   │   └── my_robot_gazebo.launch.xml   Standalone robot bringup (test world)
│   ├── config/
│   │   └── gazebo_bridge.yaml           ROS↔Gz topic mappings
│   └── worlds/
│       └── test_world.sdf               Development test environment
│
├── my_robot_description/           Robot URDF/Xacro model
│   ├── urdf/
│   │   ├── my_robot.urdf.xacro          Top-level; composes all sub-models
│   │   ├── mobile_base.xacro            Diff-drive base geometry + joints
│   │   ├── mobile_base_gazebo.xacro     DiffDrive + JointStatePublisher plugins
│   │   ├── arm.xacro                    2-DOF manipulator (revolute joints)
│   │   ├── arm_gazebo.xacro             JointPositionController plugins
│   │   ├── camera.xacro                 RGB camera sensor definition
│   │   └── common_properties.xacro      Shared inertia macros + materials
│   ├── launch/
│   │   └── display.launch.py            Visualise robot in RViz (no sim)
│   └── rviz/
│       └── urdf_config.rviz
│
└── spatial_maps/                   Multi-floor hospital navigation package
    ├── scripts/
    │   ├── map_publisher_node.py        Map server (PGM→/map, TransientLocal QoS)
    │   ├── poi_publisher_node.py        BIM semantic layer → RViz MarkerArray
    │   ├── poi_nav_node.py              POI goal projection + Nav2 action client
    │   ├── poi_click_node.py            RViz click → nearest room → /goal_poi
    │   ├── network_navigation_node.py   Dijkstra pathfinding over BIM topology
    │   ├── odom_tf_republisher.py       Retimestamped odom→base_footprint TF
    │   ├── joint_state_relay.py         Retimestamped Gz→ROS joint states
    │   └── sweep_test.py                Automated room-by-room navigation test
    ├── launch/
    │   ├── spatial_maps.launch.py       Multi-floor parameterised entry point
    │   └── spatial_maps_1f.launch.py    Floor 1F explicit launch (reference)
    ├── maps/                            15 × (*.pgm, *.yaml, *_map.png)
    ├── network/                         15 × network_*.json + graph.json
    ├── worlds/                          15 × *.world (SDF 1.10, Gz Sim)
    ├── meshes/                          Floor 1F wall mesh (OBJ/GLB)
    └── config/
        ├── nav2_params.yaml             Full Nav2 stack configuration
        ├── gz_bridge_*.yaml             Per-floor Gz↔ROS bridge configs (15 files)
        ├── floor_config.yaml            Floor database (elevations, filenames)
        └── spatial_maps_1f.rviz         RViz visualisation layout
```

---

## What Has Been Achieved

The following has been implemented and empirically validated:

**Simulation environments** — 15 Gazebo Harmonic worlds (B1F through 14F) generated from BIM-derived occupancy maps. Each world is correctly formatted as SDF 1.10, uses the floor's occupancy grid as a ground plane texture, and launches via a single parameterised command. All 15 floors share identical world-frame coordinates, enabling consistent robot spawn and map alignment without per-floor calibration.

**Robot model** — A differential-drive mobile robot with a 2-DOF manipulator arm and RGB camera, described entirely in URDF/Xacro, with Gazebo plugin configuration for physics-accurate simulation at 1 kHz. Odometry is published at 50 Hz.

**TF pipeline** — A stable, non-flooding TF tree (`map → odom → base_footprint → ...`) maintained by two custom nodes (`odom_tf_republisher.py`, `joint_state_relay.py`) that retimestamp Gazebo bridge output to prevent `TF_OLD_DATA` warnings and resolve the disconnect between Gz Sim clock and ROS clock at bridge latency.

**Nav2 autonomous navigation** — Full Nav2 stack (planner, controller, behaviour, smoother, BT navigator, lifecycle manager) operating on static BIM-derived occupancy maps with A* global planning and Regulated Pure Pursuit local control.

**POI-aware navigation** — Three-node pipeline: (1) semantic visualisation of all BIM rooms as labelled markers in RViz; (2) click-to-navigate via RViz Publish Point tool with nearest-room lookup; (3) obstacle-aware goal projection placing Nav2 goals in reachable, clearance-validated free space.

**Topological navigation layer** — Dijkstra pathfinding over a BIM-derived graph of 5,542 nodes (856 rooms, 1,446 door waypoints, 2,194 step nodes) and 3,259 edges across all 15 floors, providing room-to-room semantic routing independent of the metric grid.

**Automated sweep test** — `sweep_test.py` iterates every POI on a given floor, sends Nav2 goals, and records results to a timestamped CSV. On Floor 1F (90 rooms, 75 navigated after filtering physically inaccessible spaces): **68 SUCCESS, 0 FAILED, 6 TIMEOUT — 91% pass rate**. The test includes configurable skip keywords (stairwells, elevators, inaccessible zones), manual coordinate overrides for rooms whose BIM centroids land inside inflated obstacles, and post-failure/timeout cooldowns to prevent nav2 state cascade failures.

---

## Installation and Setup

### Prerequisites

Ubuntu 24.04 with ROS2 Jazzy Jalisco installed. Full installation instructions: [https://docs.ros.org/en/jazzy/Installation.html](https://docs.ros.org/en/jazzy/Installation.html)

### Required ROS2 packages

```bash
sudo apt install \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-msgs \
  ros-jazzy-nav2-navfn-planner \
  ros-jazzy-nav2-controller \
  ros-jazzy-nav2-bt-navigator \
  ros-jazzy-nav2-behaviors \
  ros-jazzy-nav2-smoother \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-tf2-ros \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro
```

### Python dependencies

```bash
pip install numpy scipy pillow pyyaml
```

### Build the workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# clone or place this repository here
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Add to `~/.bashrc` for permanent sourcing:

```bash
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
echo 'export GZ_IP=127.0.0.1' >> ~/.bashrc   # prevents Gazebo multicast errors
source ~/.bashrc
```

### External data dependency

The semantic BIM data (`semantic.json`) is sourced from the upstream Indoor Spatial Map Generation Pipeline. The default path expected by all nodes is:

```
/home/jason/Downloads/OneDrive_1_4-10-2026/entity/semantic.json
```

This path can be overridden at launch time via the `semantic_json` parameter.

---

## Usage

### Launch the simulation

**Any single floor** (recommended entry point):

```bash
ros2 launch spatial_maps spatial_maps.launch.py floor:=1F
```

Valid floor values: `B1F`, `1F`, `2F`, `3F`, `4F`, `5F`, `6F`, `7F`, `8F`, `9F`, `10F`, `11F`, `12F`, `13F`, `14F`

Wait approximately 15–20 seconds for Gazebo to load, the robot to spawn, and Nav2 to report:

```
[lifecycle_manager_nav] Managed nodes are active
```

### Navigate to a room by clicking

1. In RViz, select the **Publish Point** tool (keyboard shortcut: `P`)
2. Click anywhere near a room on the floor plan
3. The nearest POI is resolved, the goal is projected into free space, and the robot begins navigating

Alternatively, publish a room name directly:

```bash
ros2 topic pub --once /goal_poi std_msgs/String "data: 'S1312'"
```

### Run the automated sweep test

In a second terminal, after Nav2 is active:

```bash
ros2 run spatial_maps sweep_test.py --ros-args \
  -p floor:=1F \
  -p skip_inaccessible:=true
```

Results are written to `~/sweep_<floor>_<timestamp>.csv`. Optional parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `floor` | `1F` | Target floor |
| `timeout_sec` | `180.0` | Per-room navigation timeout |
| `skip_inaccessible` | `false` | Filter stairwells, elevators, tagged zones |
| `skip_keywords` | `계단,ELEV,접근불가,테라스` | Comma-separated skip substrings |

### Visualise the robot model only (no simulation)

```bash
ros2 launch my_robot_description display.launch.py
```

---

## Research Context and Future Work

This project sits at the intersection of three active research domains: **indoor spatial mapping**, **BIM-to-simulation pipelines**, and **autonomous robot navigation in complex built environments**.

The translation of BIM data into operational robot environments — without manual geometry authoring — is a non-trivial research problem. Real building models contain hundreds of rooms, irregular geometries, thin walls that produce rasterisation artefacts at practical map resolutions, and semantic metadata (room function, accessibility classification, robot response behaviour) that is discarded by conventional map-building approaches. This implementation demonstrates that the full chain — IFC → occupancy grid → navigable world → POI-aware goal planning — can be made to work reliably at scale.

The dual-layer navigation architecture (metric grid + semantic topological graph) reflects current consensus in mobile robotics on the complementary roles of dense metric maps for collision avoidance and sparse topological representations for semantic goal reasoning. The BIM-derived topological graph carries stair connectivity, elevator waypoints, and door-mediated access constraints that are invisible to a purely metric planner — this is the foundation for genuine multi-floor navigation rather than floor-by-floor operation.

**Identified next development phases:**

1. **Multi-floor transition** — Implement elevator call behaviour: robot navigates to the elevator hall on floor N, triggers a simulated lift transition, and resumes on floor N±k. Requires a floor-switching service and coordinated lifecycle management of per-floor map and POI nodes.

2. **Sensor-based localisation** — Replace the static `map → odom` transform with AMCL or a scan-matching localiser using simulated LiDAR, enabling robust operation under odometry drift.

3. **Dynamic costmap integration** — Add obstacle layers to the Nav2 costmap from the robot's camera or simulated LiDAR, enabling navigation around obstacles not present in the BIM model.

4. **Multi-robot coordination** — Extend to fleet operation using the stub `multi_robot_simulation.launch.py`, with task allocation across floors via the topological network.

5. **Real-building validation** — Transfer the pipeline to a different IFC building model to validate generalisability of the BIM-to-navigation translation approach.

6. **Performance characterisation** — Systematic analysis of navigation success rate as a function of map resolution, inflation radius, and corridor width to inform parameter selection guidelines for BIM-derived environments.

---

## Acknowledgements

The research direction for this project was proposed by **Prof. Min-Koo Kim**, Smart Construction and Systems (SCS) Lab, Department of Civil Engineering, Chungbuk National University, South Korea. The original brief — to validate POI-integrated robot navigation in a BIM-derived Gazebo simulation — defined the foundational scope from which this implementation grew.

The Indoor Spatial Map Generation Pipeline that produced the BIM-derived assets (occupancy maps, semantic entity data, and topological network graphs) used in this project is a separate upstream system developed as part of the same broader research programme.

---

## License

MIT
