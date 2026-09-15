# 1F simulation checks

Headless test harness used from 15 September 2026 to validate the 1F wall mesh
alignment and mesh collision, and to check navigation against Gazebo ground
truth. Run from anywhere; results go to `$TRIALS_DIR` (default `~/sim_trials`).

```bash
DRIVE_S=70 bash run_trial.sh <world variant> <label> rt,scan,contact,motion[,sweep]
```

The runner copies `worlds/1f_<variant>.world` over `spatial_maps/worlds/1f.world`
before launching, so restore the intended world afterwards
(`cp worlds/1f_mesh.world ../../worlds/1f.world`) and check `git diff`.

| Check | Script | Measures |
| --- | --- | --- |
| `rt` | `rt_sample.py` | Real-time factor from /clock over 60 s wall |
| `scan` | `scan_check.py` | LiDAR hits projected into `map`, distance to nearest occupied PGM cell |
| `contact` | `contact_test.py` | Drives at the wall ahead of spawn; Gazebo ground truth vs odometry vs LiDAR range |
| `motion` | `motion_test.py` | Open-loop spin then straight drive; true vs commanded vs odometry, and wheel joint velocities. `MOTION_ARGS="<rad/s> <m/s> <sim s>"` |
| `sweep` | `sweep_test.py` | Full 1F POI sweep, with real-time factor sampled every 10 min |

Ground truth during sweeps:

| Script | Role |
| --- | --- |
| `gt_logger.py <label> <dir>` | Passive. Logs true pose (`gz topic` dynamic_pose) and believed pose (TF `map -> base_footprint`) every 0.5 s sim, plus every NavigateToPose status change |
| `gt_analyse.py <label> <sweep csv> <dir>` | Per goal: true and believed distance to goal, true vs believed gap, travel ratio; share of true poses inside occupied cells |
| `gt_supervisor.sh <chain script>` | Attaches the logger to each sweep started by `run_trial.sh` and appends the analysis to the run's `.result` |

`bullet_friction_test.py <bullet|dart> <dir> <variants> [t0]` runs robot variants
side by side on a bare floor without ROS, driven over gz topics. It isolated the
Bullet contact-surface settings now in `my_robot_description` (see CLAUDE.md).

World variants: `mesh` = aligned mesh with collision (the live world); `boxes` =
aligned mesh visual only plus the 1108 PGM boxes; `nocoll` = aligned mesh visual
only, no boxes; `nocolldart` = `nocoll` under DART; `meshdart` and `meshdartb` =
`mesh` under DART with its ODE or Bullet collision detector (both segfault on the
wall mesh). Requires `gz` CLI, rclpy, NumPy, SciPy, Pillow, PyYAML.
