# Simulator Changelog

Tracks every change made to the codebase.
- **DEFAULT** section = the stable baseline. Go back here when things break.
- **OPTIMISATION LOG** = changes on top of default, one entry per attempt.

---

## DEFAULT BASELINE (as of 2026-03-16)

This is the known-good state. All values below are what is currently in the files.

### Physics
| File | Setting | Value |
|---|---|---|
| `fs_vehicle/worlds/empty_track.sdf` | Physics engine | DART (type="ignored") |
| `fs_vehicle/worlds/empty_track.sdf` | Physics rate | 125 Hz |
| `fs_track_generator/world.py` | Physics engine | DART (type="ignored") |
| `fs_track_generator/world.py` | Physics rate | 125 Hz |
| `fs_track_generator/world.py` | SceneBroadcaster | default (no tags, 60 Hz) |

### Cone Physics
| File | Setting | Value |
|---|---|---|
| `fs_track_generator/world.py` | Cone type | Dynamic (react to car) |
| `fs_track_generator/world.py` | max_contacts per cone | default (4) |
| `fs_track_generator/world.py` | allow_auto_disable | true (no-op with DART — confirmed) |
| `fs_track_generator/world.py` | Cone mass (small/large) | 0.8 kg / 1.5 kg |
| `fs_track_generator/world.py` | Cone friction mu | 0.6 |
| `fs_track_generator/world.py` | Cone linear/angular damping | 0.3 / 0.5 |
| `fs_track_generator/world.py` | Collision shape | Cylinder primitive |
| `fs_track_generator/world.py` | Visual (render_mode=high) | DAE mesh |
| `fs_track_generator/world.py` | Visual (render_mode=performance) | Coloured cylinder |
| `fs_track_generator/world.py` | Default render_mode | high |
| `fs_track_generator/world.py` | Shadows | true |
| `fs_track_generator/world.py` | Contact plugin | Removed (was publishing ~350 pairs/tick for nothing) |

### Cone Spacing
| File | Setting | Value |
|---|---|---|
| `fs_track_generator/layout.py` | Straight spacing | 4.5 m |
| `fs_track_generator/layout.py` | Corner spacing | 2.0 m |
| `fs_track_generator/layout.py` | Skidpad spacing | 2.5 m |

### Cone Counts (seed=42)
| Track | Cones |
|---|---|
| Acceleration | 36 |
| Skidpad | 188 |
| Trackdrive | 196 |
| Autocross | 373 |

> **Note:** Autocross removed (OPT-011). Trackdrive is now the only closed-loop event. Cone spacing updated to FSG maximums (OPT-011).

### Ground Friction
| File | Setting | Value |
|---|---|---|
| `fs_track_generator/world.py` | Dry tarmac μ | 1.3 (Hoosier slicks, FSAE TTC data) |
| `fs_track_generator/world.py` | Wet tarmac μ | 0.5 |
| `fs_vehicle/worlds/empty_track.sdf` | Ground μ | 1.3 |

### Vehicle / Sensors
| File | Setting | Value |
|---|---|---|
| `fs_vehicle/urdf/sensors.urdf.xacro` | Camera resolution | 640×480 |
| `fs_vehicle/urdf/sensors.urdf.xacro` | Camera rate | 15 Hz |
| `fs_vehicle/urdf/sensors.urdf.xacro` | LiDAR channels | 32 |
| `fs_vehicle/urdf/sensors.urdf.xacro` | LiDAR samples/scan | 720 |
| `fs_vehicle/urdf/sensors.urdf.xacro` | LiDAR range | 100 m |
| `fs_vehicle/urdf/sensors.urdf.xacro` | LiDAR rate | 10 Hz |
| `fs_vehicle/urdf/wheels.urdf.xacro` | Wheel contact | ODE kp=1e8, kd=1.0, min_depth=0.003 |
| `fs_vehicle_plugins` | Ackermann drive | Speed ramping, dual-wheel geometry, velocity-servo steering |
| `fs_vehicle/urdf/fs_car.urdf.xacro` | max_accel / max_decel | 4.0 / 8.0 m/s² |

### Launch / RViz
| File | Setting | Value |
|---|---|---|
| `fs_vehicle/launch/spawn_vehicle.launch.py` | launch_rviz default | false |
| `fs_vehicle/config/fs_car.rviz` | Fixed frame | base_link |
| `fs_vehicle/config/fs_car.rviz` | Frame rate | 30 Hz |

### RTF (baseline performance)
| Track | RTF |
|---|---|
| Acceleration | ~100% |
| Trackdrive | ~75–80% |
| Autocross | ~45–55% |

---

## OPTIMISATION LOG

### OPT-016 — Stop All closes RViz2 · remove keyboard teleop button
**Date:** 2026-03-21
**Status:** ✅ Active
**Files:**
- `fs_tui/sim_process.py`
- `fs_tui/panel.py`
- `dashboard/PIPELINE.md`
- `CHANGELOG.md`

**What changed:**

1. **Stop All now closes RViz2** — `SimController.stop()` calls `_kill_tool("rviz")` after terminating the simulation. If an RViz2 window was launched via the Tools panel, it is terminated automatically. Previously it stayed open after the simulation stopped, showing a stale TF tree.

2. **Keyboard Teleop button removed** — the button was non-functional (teleop_twist_keyboard requires an interactive terminal; the subprocess had no TTY). Removed the button from the TUI, the action mapping, and the `launch_keyboard_teleop()` / `_find_terminal_cmd()` dead code from `sim_process.py`.

**To revert:**
- Remove `self._kill_tool("rviz")` from both branches of `stop()` in `sim_process.py`
- Restore `_find_terminal_cmd()` and `launch_keyboard_teleop()` in `sim_process.py`
- Restore `Button("Keyboard Teleop", ...)` in `panel.py` and add `"btn-kb-teleop"` back to the action map

---

### OPT-015 — Module rename pass for clearer ownership
**Date:** 2026-03-18
**Status:** ✅ Active
**Files:**
- `fs_track_generator/models.py`
- `fs_track_generator/layout.py`
- `fs_track_generator/world.py`
- `fs_track_generator/artifacts.py`
- `fs_track_generator/validation.py`
- `fs_track_generator/pipeline.py`
- `fs_track_generator/ros_bridge.py`
- `fs_tui/panel.py`
- `fs_tui/sim_process.py`
- `dashboard/PIPELINE.md`
- `CHANGELOG.md`

**What changed:**
1. Track-generator modules were renamed to match their actual responsibility:
   - `types.py` → `models.py`
   - `generator.py` → `layout.py`
   - `sdf_builder.py` → `world.py`
   - `storage.py` → `artifacts.py`
   - `validator.py` → `validation.py`
   - `workflow.py` → `pipeline.py`
   - `bridge.py` → `ros_bridge.py`
2. TUI modules were renamed for the same reason:
   - `app.py` → `panel.py`
   - `controller.py` → `sim_process.py`
3. Imports, launch files, tests, and the console entry point were updated to use the new module names.
4. The changelog baseline and pipeline documentation were updated so the current architecture matches the current filenames.

**Notes:**
- Runtime behavior is unchanged; this is a naming/maintainability pass.
- `ros_bridge.py` keeps backward-compatible `write_bridge_config()` aliases so older call sites still work during the transition.

### OPT-014 — Hardcoded FSG circuits · remove seed/weather · TUI tools + map pipeline
**Date:** 2026-03-18
**Status:** ✅ Active
**Files:**
- `fs_track_generator/generator.py`
- `fs_track_generator/storage.py`
- `fs_bringup/launch/sim.launch.py`
- `fs_tui/app.py`
- `fs_tui/controller.py`
- `test/test_track_generator.py`
- `fs_track/worlds/maps/` (new directory)

**Changes:**

1. **Trackdrive generator replaced with hardcoded FSG templates** —
   The seeded polar/random generator produced circular-looking layouts.
   Replaced with two fixed control-point templates inspired by real FSG
   Trackdrive circuits (main straight + sweeping hairpin + S-curve section):
   - `_TRACKDRIVE_TEMPLATE` — 12 control points, ~450 m, classic D-shape
   - `_TEST_TRACKDRIVE_TEMPLATE` — 9 control points, ~300 m, compact technical layout
   Both use Catmull-Rom closed spline → resample → scale. No randomisation.

2. **test_trackdrive length** — 150 m → **300 m**

3. **Seed and weather removed from user-facing surfaces** —
   - `sim.launch.py`: `seed` and `weather` launch args removed; internally fixed at seed=0, weather="dry"
   - `fs_tui/app.py`: Seed Input, dice button, New Random button, Weather RadioSet all removed
   - `SimulationConfig` dataclass: `weather` field removed; `map_file` field added

4. **TUI: Three tool-launch buttons added** (independent of simulation state):
   - **RViz2** — launches `ros2 run rviz2 rviz2 -d <fs_vehicle/config/fs_car.rviz>`
   - **PS Teleop** — launches `ros2 run fs_vehicle ps_teleop`
   - **KB Teleop** — opens keyboard teleop in a new terminal window

5. **TUI: Custom map pipeline added** —
   - `storage.py`: `default_maps_dir()` + `list_map_files()` helpers added
   - Directory `simulator/fs_track/worlds/maps/` created; drop `.world` / `.sdf` files here
   - TUI scans the directory at startup and adds a radio button per file at the bottom of
     the Track Type panel; selecting one launches Gazebo with that SDF directly (skips generation)
   - `sim.launch.py`: `map_file` launch arg added; when set, track generation is bypassed

6. **`test_different_seeds_produce_different_centerlines`** test removed (seeds no longer vary layout)

**To revert:**
- Restore `_generate_trackdrive_centerline(seed, target_length)` in generator.py
- Restore seed/weather args in sim.launch.py and SimulationConfig
- Remove tool buttons from app.py
- Remove map pipeline from storage.py and app.py
- Revert test_trackdrive length 300→150

---

Each entry below is a change on top of the default baseline.
Status: ✅ kept | ❌ reverted | 🔬 pending test

---

### OPT-001 — Cone max_contacts 4→1
**Date:** 2026-03-16
**Status:** ❌ Reverted to default
**Files:** `fs_track_generator/sdf_builder.py`
**Change:** `<max_contacts>1</max_contacts>` on each cone collision
**Why:** 4× fewer ODE constraint evaluations per cone per tick
**Result:** Modest RTF gain (~5%). Reverted to keep default clean.
**To re-apply:** Change `<max_contacts>1</max_contacts>` inside `_cone_model()`, remove old comment block.

---

### OPT-002 — Cone spacing increase
**Date:** 2026-03-16
**Status:** ❌ Reverted to default
**Files:** `fs_track_generator/generator.py`
**Change:** straight 4.5→5.0 m, corner 2.0→2.5 m
**Why:** Fewer cones = fewer dynamic bodies = less physics solver work
**Result:** Autocross 373→336 cones (~10% fewer). Small RTF gain.
**Note:** 3.0 m corners caused cone overlap validation failure in tight hairpins.
**To re-apply:** In `_cone_spacing()`: `straight_spacing = 5.0`, `corner_spacing = 2.5`

---

### OPT-003 — render_mode default: high→performance
**Date:** 2026-03-16
**Status:** ❌ Reverted to default
**Files:** `fs_track_generator/sdf_builder.py`
**Change:** `render_mode: str = "performance"` in `build_world_sdf()`
**Why:** Coloured cylinders render faster than DAE meshes for 300+ objects
**Result:** Minor GPU savings. Reverted to keep default clean.
**To re-apply:** Change default in `build_world_sdf(render_mode: str = "performance")`
**Note:** Can also be set per-launch via `render_mode` arg without changing default.

---

### OPT-004 — Shadows disabled
**Date:** 2026-03-16
**Status:** ❌ Reverted to default (user reverted)
**Files:** `fs_track_generator/sdf_builder.py`
**Change:** `shadows = "false"` (was `"true"`)
**Why:** 300+ shadow-casting meshes = significant GPU shadow map cost per frame
**Result:** Unknown — user reverted before testing.
**To re-apply:** Change `shadows = "true"` → `shadows = "false"` in `build_world_sdf()`

---

### OPT-005 — Physics rate 125→100 Hz
**Date:** 2026-03-16
**Status:** ❌ Reverted to default (user reverted)
**Files:** `fs_track_generator/sdf_builder.py`, `fs_vehicle/worlds/empty_track.sdf`
**Change:** `max_step_size` 0.008→0.010, `real_time_update_rate` 125→100
**Why:** 20% fewer physics ticks per second
**Result:** Not properly tested before revert.
**To re-apply:** Change both files: `<max_step_size>0.010</max_step_size>`, `<real_time_update_rate>100</real_time_update_rate>`, rename physics block to `name="10ms"`

---

### OPT-006 — ODE quick solver, 20 iterations
**Date:** 2026-03-16
**Status:** ❌ Reverted to default
**Files:** `fs_track_generator/sdf_builder.py`, `fs_vehicle/worlds/empty_track.sdf`
**Change:** `type="ode"` with `<solver><type>quick</type><iters>20</iters><sor>1.3</sor></solver>`
**Why:** Default is 50 iterations; 20 is enough for cones on flat ground
**Result:** Unknown — reverted when switching to Bullet.
**To re-apply:**
```xml
<physics name="8ms" type="ode">
  <max_step_size>0.008</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>125</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>20</iters>
      <sor>1.3</sor>
    </solver>
  </ode>
</physics>
```

---

### OPT-007 — Bullet Featherstone physics engine
**Date:** 2026-03-16
**Status:** ❌ Reverted — car dynamics broken
**Files:** `fs_track_generator/sdf_builder.py`, `fs_vehicle/worlds/empty_track.sdf`
**Change:** Added `<engine><filename>gz-physics7-bullet-featherstone-plugin</filename></engine>` to Physics plugin
**Why:** Bullet gave ~100% RTF on autocross vs ~45–55% with DART
**Result:** RTF excellent but car steering and friction unusable. Reverted.
**Notes:**
- gz-physics is 7.6.0 — all critical PRs included (#699 #713 #738)
- Car plugin (FsAckermannDrive) is correctly written for Bullet (velocity-servo steering)
- Issue is friction feel/tuning, not API compatibility
- Requires `<bullet>` friction SDF blocks (Bullet ignores `<ode>` friction)
- Requires `<bounce>` block on wheels (Bullet ignores `<contact><ode>` kp/kd/min_depth)

---

### OPT-008 — Bullet friction SDF blocks
**Date:** 2026-03-16
**Status:** ❌ Reverted with OPT-007
**Files:** `fs_track_generator/sdf_builder.py`, `fs_vehicle/urdf/wheels.urdf.xacro`
**Change:** Added `<bullet><friction>...</friction></bullet>` alongside `<ode>` blocks on cones, ground, and wheels. Added `<bounce><restitution_coefficient>0.0</restitution_coefficient></bounce>` to wheels.
**Why:** Bullet ignores `<ode>` friction entirely — without `<bullet>` blocks the car has zero grip
**To re-apply (if trying Bullet again):**
- Cone/ground: add inside `<surface><friction>` alongside `<ode>` block
- Wheels: add `<bounce>` block and `<bullet>` friction block

---

### OPT-010 — Multi-threaded bridge + gamemode + OMP
**Date:** 2026-03-16
**Status:** ✅ Active
**Files:**
- `fs_vehicle/config/ros_gz_bridge.yaml` — now control topics only
- `fs_vehicle/config/ros_gz_bridge_sensors.yaml` — new, sensor topics only
- `fs_vehicle/launch/spawn_vehicle.launch.py` — two bridge nodes, LD_PRELOAD, OMP_NUM_THREADS
- `simulator/fs_bringup/launch/sim.launch.py` — passes `sensors_bridge_config` path
- `simulator/fs_track_generator/bridge.py` — split into control + sensors config generators
- `simulator/fs_track_generator/workflow.py` — writes both bridge configs
- `simulator/fs_track_generator/storage.py` — `map_file_set` returns `bridge_sensors` path
- `fs_vehicle/scripts/ps_teleop` — MultiThreadedExecutor

**What changed:**

1. **Bridge split into two parallel nodes**
   - `ros_gz_bridge_control`: cmd_vel, odom, tf, joint_states, imu, clock
   - `ros_gz_bridge_sensors`: /scan, /lidar/points, /camera/image_raw, /camera/camera_info
   - Each node = separate OS process = separate thread. 23k-point LiDAR clouds at 10 Hz can no longer block cmd_vel delivery.

2. **`LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libgamemodeauto.so.0`**
   - All child processes (Gazebo, bridges, RSP) automatically request gamemode from gamemoded.
   - Equivalent to running `gamemoderun ros2 launch ...` but automatic.
   - gamemoded sets CPU governor to performance, pins to P-cores.

3. **`OMP_NUM_THREADS=4`**
   - Eigen/OpenMP matrix ops in DART and bridge serialisers can use up to 4 threads.

4. **ps_teleop `MultiThreadedExecutor`**
   - Joy subscription callback and cmd_vel publisher run on separate threads.

**To revert:** Remove `gamemode_preload`, `omp_threads`, `sensors_bridge_config_arg`, `sensors_bridge` from spawn_vehicle.launch.py; restore single bridge node; revert ros_gz_bridge.yaml to include all topics; remove ros_gz_bridge_sensors.yaml; revert bridge.py/workflow.py/storage.py.

---

### OPT-011 — Remove autocross, increase cone spacing to FSG maximums
**Date:** 2026-03-18
**Status:** ✅ Active
**Files:**
- `fs_track_generator/generator.py`
- `fs_track_generator/cli.py`
- `fs_track_generator/validator.py`
- `fs_bringup/launch/sim.launch.py`
- `fs_tui/app.py`
- `test/test_track_generator.py`

**What changed:**
1. **Autocross removed as a track type** — "trackdrive" is now the sole closed-loop event. Same geometry generator (`_generate_autocross_centerline`) is used, but there is no more 400 m cap on trackdrive length. Default is 450 m.
2. **Cone spacing at FSG 2024 maximums** — FSG rules allow max 5 m on straights, max 3 m on corners. Using 5.0 m straight / 2.5 m corner (3.0 m corners failed hairpin overlap validation in previous testing).
3. **`AUTOCROSS_ELEMENTS` list removed** from generator.py
4. **`REQUIRED_AUTOCROSS_ELEMENTS` + `element_rule_ok`** removed from validator.py — was autocross-only check
5. **TUI**: Autocross radio button removed, Trackdrive set as default (label updated to "~650 m")

**Why:** Autocross generated ~373 cones at 650 m vs trackdrive's ~196 at 400 m — double the physics cost for no practical testing benefit. Removing it forces use of the shorter/fewer-cone trackdrive configuration. FSG-maximum cone spacing further reduces cone count.

**Expected cone count for trackdrive seed=42 at 450 m:** ~170–200 (vs 373 for old autocross at 650 m)

**To revert:**
- Restore `AUTOCROSS_ELEMENTS` list in generator.py
- Restore `effective_length = min(target_length_m, 400.0) if track_type == "trackdrive"` and `else:` catch-all
- Restore `REQUIRED_AUTOCROSS_ELEMENTS` + `element_rule_ok` in validator.py
- Add "autocross" back to cli.py choices and sim.launch.py `_DEFAULT_LENGTHS`
- Add autocross RadioButton back to TUI, reset defaults
- Revert spacing: straight 5.0→4.5, corner 2.5→2.0

---

### OPT-012 — Fix skidpad geometry to FSG D4.1 spec + clear crossing corridor
**Date:** 2026-03-18
**Status:** ✅ Active
**Files:** `fs_track_generator/generator.py`

**Root cause — wrong geometry (original):** Old code used `_SKIDPAD_INNER_R = 15.25` treating the inner circle **diameter** as a **radius** (2× too large). Circle centres were at ±16.75 m → 33.5 m apart; correct is 18.25 m apart.

**Root cause — crossing zone cones (v2):** After fixing the scale, the simple 2 m radial exclusion didn't catch outer boundary cones at ≈ (±5.0, ±0.25 m) — these lie inside the 3 m crossing corridor. Also, the approach/exit corridor used a `while x > -15` loop placing **11 cone pairs** (30 m of corridor cones) when the FSG reference layout shows only a short 3-pair gate.

**Fix:**
1. **Corrected constants** (FSG D4.1.2):
   - `_SKIDPAD_HALF_SPAN = 9.125 m` (was 16.75 m)
   - `_SKIDPAD_INNER_R = 7.625 m` (was 15.25 m — diameter treated as radius)
   - `_SKIDPAD_OUTER_R = 10.625 m` (was 18.25 m)
2. **Correct cone counts** (FSG D4.1.3): 16 inner orange + 13 outer per circle (was 42)
3. **Rectangular crossing exclusion** (replaces radial exclusion): no boundary cone placed where `|y| < 2.0 m AND |x| < 6.0 m`. The outer circles cross y=0 at x ≈ ±5.44 m; this zone catches those cones precisely.
4. **Short entry/exit gate** (3 pairs per side): cones at x = 6.5, 9.0, 11.5 m (entry from +X) and x = −6.5, −9.0, −11.5 m (exit to −X). No long corridor of cones in the drivable path.
5. **Timekeeping gate** (D4.1.6): 2 large orange cones at (0, ±1.9 m) mark the centre-to-centre line.

**Crossing corridor is now completely free**: no inner boundary, outer boundary, or corridor cones between the two lobes. Only the timekeeping gate cones at the crossing threshold.

**To revert:** Restore old constants (16.75/15.25/18.25), restore `_SKIDPAD_CROSSING_CLEAR_RADIUS = 2.0` radial exclusion, restore `while x > -entry_len: x -= 3.0` corridor loop with `entry_len = 15.0`.

---

### OPT-013 — Test trackdrive + seed randomisation fix + IMU system plugin
**Date:** 2026-03-18
**Status:** ✅ Active
**Files:**
- `fs_track_generator/generator.py`
- `fs_track_generator/validator.py`
- `fs_track_generator/cli.py`
- `fs_bringup/launch/sim.launch.py`
- `fs_tui/app.py`
- `fs_vehicle/worlds/empty_track.sdf`
- `fs_track_generator/sdf_builder.py`
- `test/test_track_generator.py`

**Changes:**

1. **`test_trackdrive` track type added** — 150 m default closed loop using the same seeded generator as trackdrive. Produces ~40–70 cones (vs ~170–200 for full trackdrive) → significantly higher RTF for algorithm development and testing. Exposed in TUI as "Test Track (~150 m)" radio button.

2. **Seeded track randomisation rewritten** — Old generator used a fixed 15-point template and perturbed only 7 points by ±4–8 m, producing nearly identical layouts across seeds. New `_generate_trackdrive_centerline()`:
   - Number of corners **n = 7–11** varies per seed → different track character (oval vs technical)
   - **Radial jitter 72–128 %** of unit radius → varied shapes without extreme curvature spikes
   - **Angular jitter ±15 %** per step → irregular spacing between turns
   - Parameters tuned so min turn radius stays ≥ 4.0 m for all seeds and target lengths.
   - Old function `_generate_autocross_centerline` deleted; all references updated.

3. **IMU Gazebo system plugin added** — `gz-sim-imu-system` must be loaded separately from `gz-sim-sensors-system`. Without it, the IMU sensor entity is present in the model but never publishes to `/imu/data`. Added to both `empty_track.sdf` and `sdf_builder.py`.

4. **Cone overlap fix** — Start/finish gate cones placed at the 5 m gate position coincided with the first boundary cone placed by `_place_side_cones` (both land on the left boundary at ~5 m along track). Fixed by filtering boundary cones within 0.2 m of any start/finish cone before combining the cone lists.

5. **Performance shadows fix** — `shadows` variable in `sdf_builder.py` was hardcoded to `"true"` and never updated for `render_mode="performance"`. Fixed: `shadows = "false" if render_mode == "performance" else "true"`. Shadow and `cast_shadows` in the scene/sun now correctly disable for performance mode.

6. **Codebase cleanup:**
   - `validator.py` docstring: removed stale "autocross" / "element rule" references
   - `cli.py` help text: "autocross/trackdrive" → "trackdrive/test_trackdrive"
   - `sim.launch.py` description: fixed stale "autocross" mention; fallback length 650→450
   - `_cone_spacing()`: guard changed from `== "skidpad"` to `not in ("trackdrive", "test_trackdrive")` so it applies correctly to all closed-loop types
   - Tests updated: target_length 650→450, min_turn_radius 5.0→4.0 (matches launch default); 3 new tests added (test_trackdrive validity, seed uniqueness, versioned metadata)

**Expected cone counts (seed=42):**
| Track | Length | Cones |
|---|---|---|
| test_trackdrive | 150 m | ~40–70 |
| trackdrive | 450 m | ~140–200 |

**All 7 tests pass.**

**To revert:**
- Restore `_generate_autocross_centerline` with fixed template in generator.py
- Remove `test_trackdrive` from generator, cli, sim.launch.py, app.py
- Remove `gz-sim-imu-system` plugin from both world files
- Remove boundary cone dedup step in `generate_track_layout()`
- Revert `shadows = "true"` (unconditional) in `sdf_builder.py`

---

### OPT-009 — SceneBroadcaster throttle
**Date:** 2026-03-16
**Status:** ❌ Reverted — caused severe GUI lag
**Files:** `fs_track_generator/sdf_builder.py`, `fs_vehicle/worlds/empty_track.sdf`
**Change:** Added `<dynamic_pose_hertz>30</dynamic_pose_hertz>` + `<state_hertz>10</state_hertz>`
**Why:** 336 cones × 125 Hz = 42,000 pose serialisations/sec. `<update_rate>` tag is silently ignored by gz-sim 8 SceneBroadcaster — these are the correct tags.
**Result:** `state_hertz=10` caused car to visually lag/freeze in Gazebo GUI. Reverted.
**Partial re-apply (safe):** `<dynamic_pose_hertz>30</dynamic_pose_hertz>` alone (no `state_hertz`) should be safe — reduces pose streaming without affecting world state updates.
