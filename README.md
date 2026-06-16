# Hospital Robot Navigation Simulation
### BIM-Derived Multi-Floor Autonomous Navigation with POI Integration

---

## Research Summary

This project implements a physics-based, multi-floor autonomous robot navigation system for a 15-floor hospital building, constructed entirely from Building Information Modelling (BIM) data. It was developed as an independent research initiative by Jason Mwinsegre Dassah, a Ghanaian construction engineer and robotics researcher, and is directly aligned with the research agenda of the Smart Construction and Systems (SCS) Lab at Chungbuk National University, South Korea, whose work spans construction robotics, spatial data pipelines, and simulation-based systems for smart built environments. The project has also attracted research interest from the Laboratory for Interactive Visualization in Engineering (LIVE Lab), University of Michigan, led by Prof. Vineet Kamat.

The original research brief was to generate a Gazebo simulation environment from the outputs of an Indoor Spatial Map Generation Pipeline and validate POI (Point of Interest) integration by deploying a robot on a single floor. The Indoor Spatial Map Generation Pipeline is an upstream process that parses IFC-format BIM models to extract spatial entities (IfcSpace objects), room connectivity, door positions, stairwell geometry, and floor-level occupancy grids, producing both 2D navigable maps and semantic topology graphs as structured outputs.

This implementation goes substantially beyond the original brief. Rather than validating a single floor, the system spans all 15 floors of the hospital (B1F through 14F), provides fully parameterised launch infrastructure, integrates a dual-layer navigation architecture combining grid-based and topological planning, and includes an automated 90-room sweep test with cascade recovery. Floor 2F has been empirically validated at an 83% navigation success rate. Floor 1F presents additional challenges arising from BIM-to-PGM coordinate misalignment and is under active investigation. Each floor is independently launchable using a single parameterised entry point, and every floor carries its own occupancy map, semantic POI layer, topological network graph, and Gazebo simulation world.

This work demonstrates that IFC/BIM spatial data can be translated into a fully operational, multi-floor robot navigation stack without manual environment modelling, and that POI-aware goal-directed navigation in complex built environments is achievable using open-source robotics middleware at research quality.

---

## System Architecture

The pipeline flows from raw BIM data through to autonomous robot behaviour in simulation:

```
 IFC BIM Model (upstream pipeline)
        |
        +-- semantic.json          per-floor IfcSpace entities, centroids,
        |                          display names, robot_response metadata
        |
        +-- maps/*.pgm + *.yaml    2D occupancy grids (0.02 m/px) for each
        |                          floor, compatible with ROS2 map server
        |
        +-- network/*.json         topological graphs: room nodes, door nodes,
                                   step nodes, Dijkstra-traversable edges
                                   (5,542 nodes, 3,259 edges, 15 floors)
                                           |
                                           v
 +---------------------------------------------------------------+
 |                    Gazebo Harmonic (Gz Sim 8)                 |
 |  15 x SDF 1.10 world files, one per floor                    |
 |  Floor plan texture (PGM->PNG), inline directional lighting   |
 |  Robot spawned at (21.0, 38.0, 0.1) in map frame             |
 |  1F: Bullet Featherstone physics, BIM wall mesh (visual),    |
 |      invisible perimeter fence (collision-only, 4 box walls)  |
 +----------------------------+----------------------------------+
                              | ros_gz_bridge (per-floor YAML config)
                              v
 +---------------------------------------------------------------+
 |                    my_robot_description                       |
 |  Differential-drive base (0.6 x 0.4 x 0.2 m)                |
 |  2-DOF arm (revolute joints, 0 to 90 deg, position-control)  |
 |  RGB camera (640x480, 80 deg FOV, 20 Hz)                     |
 |  360-degree GPU LiDAR (12 m range, 10 Hz, 360 samples)       |
 |  Odometry at 50 Hz via DiffDrive Gz plugin                   |
 +----------------------------+----------------------------------+
                              |
          +-------------------+-------------------+
          v                   v                   v
  odom_tf_republisher   joint_state_relay   robot_state_publisher
  (odom->base_footprint  (retimestamps Gz    (URDF->TF tree,
   TF, retimestamped,     joint states,       /robot_description)
   monotonic guard)       monotonic guard)
          |                                       |
          +------------------+--------------------+
                             v
 +---------------------------------------------------------------+
 |                       Nav2 Stack                             |
 |  map_publisher_node   ->  /map  (TransientLocal QoS,         |
 |                           1s startup timer, 0.001 Hz repeat) |
 |  amcl                 ->  particle filter localisation        |
 |                           (tf_broadcast: false; static TF     |
 |                            used as primary map->odom source)  |
 |  static_tf_publisher  ->  map->odom at spawn (21, 38, 0.1)   |
 |  planner_server       ->  NavfnPlanner / A* (0.5 m tol.)    |
 |  controller_server    ->  RegulatedPurePursuit (0.26 m/s)    |
 |  behavior_server      ->  spin, backup, drive_on_heading,    |
 |                           wait                               |
 |  smoother_server      ->  path smoothing                     |
 |  bt_navigator         ->  NavigateToPose / NavigateThroughP. |
 |  lifecycle_manager    ->  autostart, 12s delay, bond 30s     |
 +----------------------------+---------------------------------+
                              ^
          +-------------------+------------------+
          |                                      |
 +--------+---------+                +-----------+---------+
 |   POI Layer      |                |  Topological Layer  |
 |                  |                |                     |
 | poi_publisher    |                | network_navigation  |
 |  (sphere+text    |                |  _node              |
 |   markers in     |                |  (Dijkstra over     |
 |   RViz, colour-  |                |   BIM-derived graph,|
 |   coded by       |                |   5,542 nodes)      |
 |   robot_response |                |                     |
 |   class)         |                +---------------------+
 |                  |
 | poi_click_node   |  <- RViz Publish Point click -> nearest room
 | poi_nav_node     |  -> goal projection (EDT + BFS) -> Nav2 goal
 +------------------+
```

### Goal Projection

A critical implementation detail is the goal projection algorithm in `poi_nav_node.py`. BIM room centroids frequently fall inside walls or inflation-layer obstacles. The algorithm projects each centroid outward using:

1. **Reachability BFS**: flood-fill from the robot spawn point at launch to identify all connected free cells.
2. **Euclidean Distance Transform** (scipy `distance_transform_edt`): pre-computes wall clearance for every map cell.
3. **Two-pass BFS projection**: first searches free-space-only; if no valid cell is found within 10 m, a second pass allows wall-crossing to guarantee a result.
4. **Clearance threshold**: 0.65 m (inflation radius 0.55 m + 0.10 m buffer) to prevent goals adjacent to inflated obstacles.

### Cascade Recovery

`sweep_test.py` monitors for a stuck-robot pattern: three consecutive FAILED navigation results each completing in under 30 seconds indicates that the planner is immediately rejecting goals due to an occupied start position (caused by odometry drift through visual walls). When this pattern is detected, the sweep test sends the robot back to the known-good spawn point before resuming. Up to five such recoveries are attempted per sweep, each logged as a distinct row in the output CSV.

### Localization Architecture

Localization uses a layered fallback approach. A static `map->odom` transform published at the robot's spawn position (21.0, 38.0, 0.1) provides the primary map frame. AMCL runs concurrently, subscribing to `/scan` from the GPU LiDAR, but `tf_broadcast` is set to `false` to prevent AMCL from overriding the static TF. This conservative configuration was necessary because the 1F BIM wall mesh has an approximate 0.9 m coordinate offset from the PGM map, causing AMCL scan-matching to produce incorrect corrections. Resolving this alignment issue and re-enabling AMCL TF broadcast is an active research objective.

---

## Technical Stack

| Component | Technology |
|-----------|-----------|
| Robotics middleware | ROS2 Jazzy Jalisco |
| Physics simulation | Gazebo Harmonic (Gz Sim 8), SDF 1.10 |
| Physics engine (1F) | Bullet Featherstone (gz-physics7-bullet-featherstone-plugin) |
| Autonomous navigation | Nav2 (navfn A\*, RegulatedPurePursuit, BehaviorTree) |
| Localisation | AMCL (configured, tf_broadcast: false) + static map->odom TF |
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
+-- my_robot_bringup/               Robot launch and Gz bridge config
|   +-- launch/
|   |   +-- my_robot_gazebo.launch.xml   Standalone robot bringup (test world)
|   +-- config/
|   |   +-- gazebo_bridge.yaml           ROS<->Gz topic mappings (test world)
|   +-- worlds/
|       +-- test_world.sdf               Development test environment
|
+-- my_robot_description/           Robot URDF/Xacro model
|   +-- urdf/
|   |   +-- my_robot.urdf.xacro          Top-level; composes all sub-models
|   |   +-- mobile_base.xacro            Diff-drive base geometry + joints
|   |   +-- mobile_base_gazebo.xacro     DiffDrive + JointStatePublisher plugins
|   |   +-- arm.xacro                    2-DOF manipulator (revolute joints)
|   |   +-- arm_gazebo.xacro             JointPositionController plugins
|   |   +-- camera.xacro                 RGB camera sensor definition
|   |   +-- lidar.xacro                  360-degree GPU LiDAR sensor (NEW)
|   |   +-- common_properties.xacro      Shared inertia macros + materials
|   |   +-- standalone_arm.urdf.xacro    Standalone arm for testing
|   +-- launch/
|   |   +-- display.launch.py            Visualise robot in RViz (no sim)
|   +-- rviz/
|       +-- urdf_config.rviz
|
+-- spatial_maps/                   Multi-floor hospital navigation package
    +-- scripts/
    |   +-- map_publisher_node.py        Map server (PGM->/map, TransientLocal
    |   |                                QoS, 1s startup timer, 0.001 Hz repeat)
    |   +-- poi_publisher_node.py        BIM semantic layer -> RViz MarkerArray
    |   +-- poi_nav_node.py              POI goal projection + Nav2 action client
    |   +-- poi_click_node.py            RViz click -> nearest room -> /goal_poi
    |   +-- network_navigation_node.py   Dijkstra pathfinding over BIM topology
    |   +-- odom_tf_republisher.py       Retimestamped odom->base_footprint TF
    |   +-- joint_state_relay.py         Retimestamped Gz->ROS joint states
    |   +-- sweep_test.py                Automated room-by-room navigation test
    |   |                                with cascade recovery to spawn
    |   +-- pgm_to_sdf_walls.py          Utility: PGM -> Gazebo SDF collision
    |                                    wall boxes (pixel-accurate alignment)
    +-- launch/
    |   +-- spatial_maps.launch.py       Multi-floor parameterised entry point
    |   +-- spatial_maps_1f.launch.py    Floor 1F explicit launch (reference)
    |   +-- floor_1f.launch.py           )
    |   +-- floor_2f.launch.py           ) Per-floor explicit launch files,
    |   +-- ...                          ) one per floor (B1F, 1F through 14F)
    |   +-- floor_b1f.launch.py          )
    |   +-- multi_robot_simulation.launch.py  Multi-robot stub (B1F)
    +-- maps/                            15 x (*.pgm, *.yaml, *_map.png)
    +-- network/                         15 x network_*.json, graph.json,
    |                                    summary.yaml
    +-- worlds/                          15 x *.world (SDF 1.10, Gz Sim)
    +-- meshes/                          Floor 1F wall mesh (OBJ/GLB, visual only)
    +-- models/
    |   +-- pgm_walls_1f/                Gazebo model: 1108 PGM-derived collision
    |       +-- model.config             boxes for Floor 1F (generated by
    |       +-- model.sdf                pgm_to_sdf_walls.py, not loaded by default)
    +-- config/
        +-- nav2_params.yaml             Full Nav2 + AMCL stack configuration
        +-- gz_bridge_*.yaml             Per-floor Gz<->ROS bridge configs (15 files)
        +-- floor_config.yaml            Floor database (elevations, filenames)
        +-- spatial_maps_1f.rviz         RViz visualisation layout
```

---

## What Has Been Achieved

The following has been implemented and empirically validated:

**Simulation environments**: 15 Gazebo Harmonic worlds (B1F through 14F) generated from BIM-derived occupancy maps. Each world is correctly formatted as SDF 1.10, uses the floor's occupancy grid as a ground plane texture, and launches via a single parameterised command. All 15 floors share identical world-frame coordinates, enabling consistent robot spawn and map alignment without per-floor calibration. Floor 1F uses the Bullet Featherstone physics engine to support a 17,051-face BIM-derived wall mesh that crashes the default DART engine.

**Robot model**: A differential-drive mobile robot with a 2-DOF manipulator arm, RGB camera, and 360-degree GPU LiDAR, described entirely in URDF/Xacro with Gazebo plugin configuration for physics-accurate simulation at 1 kHz. Odometry is published at 50 Hz. The GPU LiDAR provides 360 samples at 10 Hz with a 12 m maximum range, and is bridged to ROS2 as a `sensor_msgs/LaserScan` on `/scan`.

**TF pipeline**: A stable, non-flooding TF tree (`map -> odom -> base_footprint -> ...`) maintained by two custom nodes (`odom_tf_republisher.py`, `joint_state_relay.py`) that retimestamp Gazebo bridge output to prevent `TF_OLD_DATA` warnings and resolve the disconnect between Gz Sim clock and ROS clock at bridge latency.

**Localization**: A layered localization approach combining a static `map->odom` transform (published at spawn position 21.0, 38.0) with AMCL configured conservatively (`tf_broadcast: false`). AMCL is fully operational and subscribes to `/scan`, but does not override the static TF because the 1F BIM wall mesh has approximately 0.9 m coordinate offset from the PGM map, which would cause AMCL corrections to be incorrect. The `bond_timeout` for the Nav2 lifecycle manager was increased from the default 4 s to 30 s to prevent AMCL crash-loops caused by the 8.26 M cell (2075 x 3982 px) map blocking the executor for longer than the default heartbeat window.

**Floor 1F physical boundary**: An invisible perimeter fence (four collision-only box walls placed 1 m outside each PGM map edge) prevents the robot from physically escaping the hospital footprint during navigation, while having no effect on the Nav2 costmap, which is static-layer only. The BIM wall mesh is rendered visually for scene realism but has no collision geometry, because its coordinate offset from the PGM would place physical obstacles inside costmap inflation zones and trap the robot.

**PGM-to-SDF wall generator**: `pgm_to_sdf_walls.py` is a standalone utility that reads any PGM occupancy map and generates a Gazebo SDF model containing pixel-accurate collision wall boxes. At 3x downsampling (0.06 m/px), it produces 1,108 boxes for the 1F map in under one second. The generated model (`models/pgm_walls_1f/`) is available as a reference but is not loaded in the default simulation due to Gazebo startup time constraints with this many physics bodies.

**Nav2 autonomous navigation**: Full Nav2 stack (planner, controller, behaviour, smoother, BT navigator, lifecycle manager) operating on static BIM-derived occupancy maps with A* global planning and Regulated Pure Pursuit local control at 0.26 m/s.

**Map publisher**: A custom map publisher node publishes the PGM occupancy grid with TRANSIENT_LOCAL QoS durability so late-subscribing Nav2 nodes receive the map correctly. A one-second startup timer ensures the map is published immediately at launch rather than waiting for the first periodic tick (0.001 Hz), preventing the AMCL initialisation failure that would otherwise occur when the map is not yet available at t~13 s when the lifecycle manager activates nodes.

**POI-aware navigation**: Three-node pipeline: (1) semantic visualisation of all BIM rooms as labelled markers in RViz, colour-coded by robot_response class (A: blue autonomous patrol, B: green passive monitoring, C: yellow interactive, H: orange human-sensitive); (2) click-to-navigate via RViz Publish Point tool with nearest-room lookup; (3) obstacle-aware goal projection placing Nav2 goals in reachable, clearance-validated free space.

**Topological navigation layer**: Dijkstra pathfinding over a BIM-derived graph of 5,542 nodes (856 rooms, 1,446 door waypoints, 2,194 step nodes) and 3,259 edges across all 15 floors, providing room-to-room semantic routing independent of the metric grid.

**Automated sweep test with cascade recovery**: `sweep_test.py` iterates every POI on a given floor, sends Nav2 goals, and records results to a timestamped CSV. Configurable skip keywords filter physically inaccessible spaces (stairwells, elevators, tagged zones). Manual coordinate overrides handle rooms whose BIM centroids land inside inflated obstacles. Cascade recovery detects stuck-robot episodes from the failure signature (three consecutive FAILEDs under 30 s) and navigates the robot back to the spawn point before resuming, breaking cascading failures caused by odometry drift into occupied costmap cells.

**Empirical results**: Floor 2F has been validated at an 83% navigation success rate across the floor's full POI set. Floor 1F achieves 38% in the current configuration; this lower rate is attributable to the BIM-to-PGM coordinate misalignment and is the primary focus of ongoing investigation. Both measurements were obtained using the automated sweep test.

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
  ros-jazzy-nav2-amcl \
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
echo 'export GZ_IP=127.0.0.1' >> ~/.bashrc
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

Wait approximately 15-20 seconds for Gazebo to load, the robot to spawn, and Nav2 to report:

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
| `timeout_sec` | `180.0` | Per-room navigation timeout (seconds) |
| `skip_inaccessible` | `false` | Filter stairwells, elevators, tagged zones |
| `skip_keywords` | `계단,ELEV,접근불가,테라스` | Comma-separated skip substrings |
| `cascade_threshold` | `3` | Consecutive fast-FAILs before recovery |
| `cascade_max_sec` | `30.0` | Max duration (s) to count a FAIL as fast |
| `recovery_timeout_sec` | `120.0` | Timeout for recovery navigation to spawn |
| `max_recoveries` | `5` | Maximum cascade recoveries per sweep |

### Generate PGM-derived wall collision geometry

To regenerate the SDF collision model for a floor from its PGM map:

```bash
python3 spatial_maps/scripts/pgm_to_sdf_walls.py \
  --yaml  spatial_maps/maps/1F.yaml \
  --output spatial_maps/worlds/pgm_walls_1f.sdf \
  --downsample 3
```

This produces 1,108 collision boxes at 0.06 m resolution for the 1F map in under one second.

### Visualise the robot model only (no simulation)

```bash
ros2 launch my_robot_description display.launch.py
```

---

## Research Context and Future Work

This project sits at the intersection of three active research domains: indoor spatial mapping, BIM-to-simulation pipelines, and autonomous robot navigation in complex built environments.

The translation of BIM data into operational robot environments without manual geometry authoring is a non-trivial research problem. Real building models contain hundreds of rooms, irregular geometries, thin walls that produce rasterisation artefacts at practical map resolutions, and semantic metadata (room function, accessibility classification, robot response behaviour) that is discarded by conventional map-building approaches. This implementation demonstrates that the full chain from IFC through occupancy grid to navigable world and POI-aware goal planning can be made to work reliably at scale.

A specific technical challenge that has emerged is the coordinate misalignment between BIM-exported geometry and PGM-derived occupancy maps. The 1F wall mesh was exported from the BIM model in a coordinate frame approximately 0.9 m offset from the PGM origin, with a scale discrepancy that prevents correction by a single rigid transform. This misalignment prevents physical collision geometry from being used alongside the Nav2 static costmap without trapping the robot inside inflation zones, and it prevents AMCL from localizing correctly because the LiDAR-visible geometry does not match the PGM occupied cells. Resolving this requires re-exporting wall geometry from the BIM model in the same coordinate frame used to generate the PGM, which is the highest-priority outstanding technical item.

The dual-layer navigation architecture (metric grid combined with semantic topological graph) reflects current consensus in mobile robotics on the complementary roles of dense metric maps for collision avoidance and sparse topological representations for semantic goal reasoning. The BIM-derived topological graph carries stair connectivity, elevator waypoints, and door-mediated access constraints that are invisible to a purely metric planner; this is the foundation for genuine multi-floor navigation rather than floor-by-floor operation.

**Identified next development phases:**

1. **BIM-to-PGM coordinate alignment**: Re-export wall geometry from the BIM model in the same coordinate frame as the PGM occupancy maps. This is a prerequisite for enabling physical wall collision that matches the costmap, allowing AMCL to localise correctly and achieving a meaningful pass rate improvement on Floor 1F.

2. **Full AMCL localisation**: Once wall alignment is resolved, re-enable `tf_broadcast: true` in AMCL so that the particle filter provides continuous pose correction during navigation, eliminating odometry drift as a source of cascading navigation failures.

3. **Multi-floor transition**: Implement elevator call behaviour so the robot navigates to the elevator hall on floor N, triggers a simulated lift transition, and resumes on floor N plus or minus k. This requires a floor-switching service and coordinated lifecycle management of per-floor map and POI nodes.

4. **Dynamic costmap integration**: Add obstacle layers to the Nav2 costmap from the GPU LiDAR, enabling navigation around obstacles not present in the BIM model. The infrastructure (LiDAR sensor, `/scan` bridge, AMCL node) is already in place; costmap configuration is the remaining step.

5. **Multi-robot coordination**: Extend to fleet operation using `multi_robot_simulation.launch.py` as a base, with task allocation across floors via the topological network.

6. **Real-building validation**: Transfer the pipeline to a different IFC building model to validate generalisability of the BIM-to-navigation translation approach.

7. **Performance characterisation**: Systematic analysis of navigation success rate as a function of map resolution, inflation radius, and corridor width to inform parameter selection guidelines for BIM-derived environments.

---

## Acknowledgements

The research direction for this project was proposed by **Prof. Min-Koo Kim**, Smart Construction and Systems (SCS) Lab, Department of Civil Engineering, Chungbuk National University, South Korea. The original brief to validate POI-integrated robot navigation in a BIM-derived Gazebo simulation defined the foundational scope from which this implementation grew.

The project has attracted research interest from the Laboratory for Interactive Visualization in Engineering (LIVE Lab) at the University of Michigan, led by **Prof. Vineet Kamat**, whose work on construction robotics and virtual environments is closely aligned with this research direction.

The Indoor Spatial Map Generation Pipeline that produced the BIM-derived assets (occupancy maps, semantic entity data, and topological network graphs) used in this project is a separate upstream system developed as part of the same broader research programme.

---

## License

MIT
