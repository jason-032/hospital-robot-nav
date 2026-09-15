# 1F simulation checks

Headless test harness used on 15 September 2026 to validate the 1F wall mesh
alignment and mesh collision. Run from anywhere; results go to `$TRIALS_DIR`
(default `~/sim_trials`).

```bash
DRIVE_S=70 bash run_trial.sh <mesh|boxes|nocoll> <label> rt,scan,contact[,sweep]
```

The runner copies `worlds/1f_<mode>.world` over `spatial_maps/worlds/1f.world`
before launching, so restore the intended world afterwards
(`cp worlds/1f_mesh.world ../../worlds/1f.world`) and check `git diff`.

| Check | Script | Measures |
| --- | --- | --- |
| `rt` | `rt_sample.py` | Real-time factor from /clock over 60 s wall |
| `scan` | `scan_check.py` | LiDAR hits projected into `map`, distance to nearest occupied PGM cell |
| `contact` | `contact_test.py` | Drives at the wall ahead of spawn; Gazebo ground truth vs odometry vs LiDAR range |
| `sweep` | `sweep_test.py` | Full 1F POI sweep, with real-time factor sampled every 10 min |

World variants: `mesh` = aligned mesh with collision; `boxes` = aligned mesh
visual only plus the 1108 PGM boxes; `nocoll` = aligned mesh visual only, no boxes.
Requires `gz` CLI, rclpy, NumPy, SciPy, Pillow.
