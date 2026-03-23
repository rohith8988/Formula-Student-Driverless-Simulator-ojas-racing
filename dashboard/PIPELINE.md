# Project Brain Dump — FS Simulator

> Written for future-me after time away. Assumes you remember ROS2 basics
> but have forgotten how the pieces connect. Every gotcha that took more than
> 30 minutes to debug is in here.

---

## Project Overview

This is a **Formula Student Driverless simulator** built on ROS2 Jazzy + Gazebo Harmonic (gz-sim 8).
It generates closed-loop race tracks procedurally, spawns a 4-wheeled FS car into them, and
exposes all sensors over standard ROS2 topics so an autonomous stack can be dropped in.

**Three ROS2 packages:**

| Package | Language | Purpose |
|---|---|---|
| `simulator` | Python | Track generator, TUI, bringup launch |
| `fs_vehicle` | C++ + Python | Car URDF, world SDF, launch, bridge configs |
| `fs_vehicle_plugins` | C++ | Gazebo system plugin — Ackermann drive |

**Entry points:**
```bash
ros2 run simulator fs_tui          # Terminal control panel (recommended)
ros2 launch simulator sim.launch.py track_type:=trackdrive
ros2 launch simulator sim.launch.py map_file:=/path/to/custom.world
ros2 launch fs_vehicle spawn_vehicle.launch.py  # Car only, static world
```

---

## Architecture & Data Flow

```
┌────────────────────────────────────────────────────────────────────────┐
│  User  →  TUI (Textual app)  →  SimController  →  ros2 launch          │
│                                                        │               │
│                                              sim.launch.py             │
│                                                   OpaqueFunction       │
│                                                        │               │
│                          ┌─────────────────────────────▼────────────┐  │
│                          │        TRACK GENERATOR PIPELINE          │  │
│                          │  map_file set? ──YES──► Gazebo directly  │  │
│                          │       │                                  │  │
│                          │       NO                                 │  │
│                          │       ▼                                  │  │
│                          │  generate_versioned_track()              │  │
│                          │    1. generate_track_layout()            │  │
│                          │       hardcoded FSG template             │  │
│                          │       centripetal Catmull-Rom spline     │  │
│                          │       resample @ 1.25m                   │  │
│                          │       build_boundaries() ±1.5m           │  │
│                          │       place_cones() (adaptive spacing)   │  │
│                          │       start_finish_cones()               │  │
│                          │    2. validate_track_layout()            │  │
│                          │    3. write artifacts to disk:           │  │
│                          │       .world  _cones.yaml  _meta.yaml    │  │
│                          │       _bridge.yaml  _bridge_sensors.yaml │  │
│                          └─────────────────────────────┬────────────┘  │
│                                                        │               │
│                                           spawn_vehicle.launch.py      │
│                                           ┌─────────────▼────────────┐ │
│                                           │  Gazebo Sim (gz-sim 8)   │ │
│                                           │  loads .world file       │ │
│                                           │  cone models embedded    │ │
│                                           │  physics: DART @ 125 Hz  │ │
│                                           └──────────┬───────────────┘ │
│                                                      │                 │
│              ┌───────────────────────────────────────┼──────────────┐  │
│              │           ros_gz_bridge (x2)          │              │  │
│              │  control:  cmd_vel, odom, tf, imu,    │              │  │
│              │            joint_states, clock        │              │  │
│              │  sensors:  /scan, /lidar/points,      │              │  │
│              │            /camera/image_raw          │              │  │
│              └───────────────────────────────────────┘              │  │
│                                                                        │
│              robot_state_publisher  ←  /robot_description (xacro)      │ 
│              RViz2  (optional, delayed 6s)                             │
└────────────────────────────────────────────────────────────────────────┘
```

### Sequence on every launch

1. **TUI → `SimController.launch()`** — assembles `ros2 launch simulator sim.launch.py ...` command (including `map_file:=…` if a custom map was selected), spawns it as an async subprocess with `start_new_session=True` (process group).
2. **`sim.launch.py` `OpaqueFunction`** — if `map_file` is set, bypasses the generator and directly returns `IncludeLaunchDescription` for Gazebo. Otherwise calls `generate_versioned_track()` **in-process** (no subprocess, no ROS node — pure Python in the launch interpreter). Takes ~50–200 ms.
3. **`generate_versioned_track()`** — generates geometry from hardcoded FSG template, validates, writes 5 artifact files to `simulator/fs_track/worlds/generated/`, returns paths + metadata.
4. **`spawn_vehicle.launch.py`** gets the world path, spawn pose, world_name, and both bridge config paths as launch arguments.
5. **Gazebo** loads the `.world` file. The world contains every cone as a `<model>` with inline geometry + physics. No external model URIs needed for performance mode; DAE meshes are `file://` URIs in high mode.
6. **`robot_state_publisher`** runs `xacro fs_car.urdf.xacro` and publishes `/robot_description`.
7. **`ros_gz_sim create`** (delayed 0.1s) reads `/robot_description` and spawns the car at the generated start pose.
8. **Two bridge nodes** map Gazebo topics ↔ ROS2 topics. The joint_state topic is world+model-name-specific; that's why the bridge config is generated per-launch.
9. **TUI** waits 5s then reads `_meta.yaml` to get start pose, world name, cone count. Status box updates.
10. **On shutdown** — ROS2 `OnShutdown` event calls `delete_generated_artifacts()` → removes all 5 files.

---

## File & Module Breakdown

### `simulator/fs_track_generator/models.py`

**Problem it solves:** Shared data structures for the whole pipeline — avoids passing raw dicts around.

**What breaks without it:** Every other module imports from here. Nothing compiles/runs.

**Key types:**

```python
@dataclass(slots=True)
class Cone:
    x: float; y: float; color: str; size: str = "small"
    # color: "blue" | "yellow" | "orange"
    # size:  "small" | "large"

@dataclass(slots=True)
class StartPose:
    x: float; y: float; z: float; yaw: float   # yaw in radians

@dataclass(slots=True)
class TrackLayout:
    # Everything the rest of the pipeline needs
    track_type: str          # "trackdrive" | "test_trackdrive" | "skidpad" | "acceleration"
    seed: int
    target_length_m: float
    centerline: list[tuple[float, float]]   # resampled at 1.25m spacing
    left_boundary: list[tuple[float, float]]  # centerline + normal * 1.5m
    right_boundary: list[tuple[float, float]] # centerline - normal * 1.5m
    cones: list[Cone]
    start_pose: StartPose
    track_length_m: float    # actual length, may differ from target
    included_elements: list[str]
    weather: str             # "dry" | "wet"
    cone_noise: str          # "low" | "none" (cone noise not yet implemented)
```

**Gotcha:** `slots=True` means you CANNOT add attributes dynamically. Any `layout.foo = bar` will raise `AttributeError`. This is intentional — keeps the API explicit.

---

### `simulator/fs_track_generator/layout.py`

**Problem it solves:** All track geometry. The only file that knows what a "Formula Student track" means geometrically.

**What breaks without it:** Nothing runs at all.

**Key functions:**

```python
# ── Hardcoded FSG circuit templates ─────────────────────────────────────────
_TRACKDRIVE_TEMPLATE      # 14-point normalized polygon → ~450 m after scaling
_TEST_TRACKDRIVE_TEMPLATE # 12-point kidney shape → ~300 m after scaling
# Points are in a unit-ish space; _load_circuit_centerline() scales them to
# target_length_m via a uniform scale factor.

def _load_circuit_centerline(template, target_length_m)
    # 1. centripetal Catmull-Rom → smooth closed curve through template points
    # 2. translate so curve[0] == (0, 0)
    # 3. scale uniformly to match target_length_m
    # 4. resample at 1.25m spacing
    # Returns list of (x, y) tuples

def _catmull_rom_closed(points, samples_per_segment=40, alpha=0.5)
    # Centripetal Catmull-Rom (Barry & Goldman algorithm, α=0.5)
    # Unlike uniform Catmull-Rom, centripetal parameterisation prevents
    # curvature spikes when adjacent control points are unevenly spaced.
    # 40 samples per segment; appends points[0] at end → closed loop.

def _resample_polyline(points, spacing=1.25)
    # Walk along the polyline, place a new point every `spacing` metres
    # Closes the loop: appends resampled[0] if last != first by > 1e-6m

def _build_boundaries(points, half_width_m=1.5)
    # Each centerline point → normal direction → offset ±1.5m
    # Returns (left_boundary, right_boundary), both same length as centerline

def _cone_spacing(points, index, track_type)
    # Returns adaptive spacing: 5.0m on straights, 2.5m on corners
    # Uses local curvature (circumradius of 3 consecutive points)
    # blend = min(curvature, 0.25) / 0.25  →  0 = straight, 1 = corner

def _place_cones(centerline, left_boundary, right_boundary, track_type)
    # Calls _place_side_cones for blue (left) then yellow (right)
    # Returns ALL boundary cones, excluding start/finish zone

def _start_finish_cones(centerline, left_boundary, right_boundary)
    # 2 large orange at boundary[0] (start gate)
    # 2 large orange at 5m along track (second gate)
    # 2 small orange flanking the start line
    # Returns 6 cones total

def generate_track_layout(*, track_type, seed, target_length_m, ...)
    # THE main entry point. Dispatches to the right generator.
    # IMPORTANT: internally calls validate_track_layout() and RAISES if invalid
    # This means bad geometry throws ValueError, not a valid=False layout.
    # Default min_turn_radius_m=4.0 (aligned with sim.launch.py).
```

**Easy to misunderstand:**

- `_local_curvature()` returns **1/R** (curvature). `_circumcircle_radius()` in `validation.py` returns **R** (radius). They use the same math, different return values.
- `boundary[-1] == boundary[0]` — the closed centerline makes the last boundary point equal to the first. `_place_side_cones` processes this last near-zero segment, potentially placing a cone right at the start gate position. This is handled by the dedup filter after the call.
- The generator runs **validation internally** and raises. Any parameter combination that produces turn radius < `min_turn_radius_m` throws `ValueError` — not a silent `valid=False`.
- **Centripetal vs uniform Catmull-Rom**: Uniform (α=0) parameterisation creates curvature spikes when a very short segment (e.g., 0.07 units) sits next to a very long one (e.g., 1.1 units). Centripetal (α=0.5) parameterises by √(chord_length), automatically weighting the tangent direction by distance. This was needed to prevent a ≈0.65m radius spike where the return straight meets the main straight.

**The dedup filter (added to fix cone overlap):**
```python
sf_cones = _start_finish_cones(...)
sf_positions = [(c.x, c.y) for c in sf_cones]
boundary_cones = _place_cones(...)
cones = [
    c for c in boundary_cones
    if all(_distance((c.x, c.y), sfp) >= 0.2 for sfp in sf_positions)
] + sf_cones
```
This runs O(n×6) per boundary cone — completely fine for ~200 cones.

---

### `simulator/fs_track_generator/validation.py`

**Problem it solves:** Ensures the generated track meets FSG rules and won't cause physics chaos (overlapping cones, impossible turns, etc.).

**What breaks without it:** Bad tracks silently get written to disk and loaded into Gazebo. Tight corners cause the car to clip through cones. Overlapping cones cause physics explosions.

**Checks performed:**

| Check | What it catches | Skipped for |
|---|---|---|
| `width_ok` | Boundary width ≠ 3.0m ± 0.15m | skidpad, acceleration |
| `radius_ok` | Turn radius < min_turn_radius_m | — |
| `closed_loop_ok` | Start/end > 2.0m apart | acceleration, skidpad |
| `self_intersection_ok` | Centerline crosses itself | skidpad (figure-8 is intentional) |
| `spacing_rule_ok` | Cone spacing outside 1.4–5.2m | skidpad, acceleration |
| `overlaps_ok` | Any two cones < 0.2m apart | skidpad |

**Called twice:** once inside `generate_track_layout()` (raises on failure) and once in `generate_versioned_track()` (result stored in metadata). The second call is redundant but harmless — it stores the validation report in the meta file for debugging.

**Gotcha:** `spacing_rule_ok` only checks **consecutive** same-color cones in list order. It does NOT wrap from last to first. So the gap between the last cone before the start/finish line and the first cone after it is never checked. This is intentional — the gap exists because of the start/finish gate zone.

---

### `simulator/fs_track_generator/world.py`

**Problem it solves:** Converts a `TrackLayout` Python object into a valid Gazebo Harmonic SDF XML string.

**What breaks without it:** No world file → Gazebo can't start → nothing works.

**Key logic:**

```python
def build_world_sdf(layout: TrackLayout, *, world_name="fs_world", render_mode="high") -> str:
    # Switches shadows, cast_shadows, and cone visual based on render_mode
    # shadows = "false" if render_mode == "performance" else "true"
    # Performance mode: cylinder geometry (no mesh URIs, faster GPU)
    # High mode: .dae mesh via file:// URI

def _cone_model(cone: Cone, index: int, mesh_dir: Path, render_mode: str) -> str:
    # Returns a complete <model> XML block for one cone
    # Includes: <static>false</static> (dynamic — reacts to car)
    # mass: 0.8kg small / 1.5kg large
    # damping: linear=0.3, angular=0.5 (keeps knocked cones from rolling forever)
    # friction mu=0.6

def _cone_mesh_name(cone: Cone) -> str:
    # Returns "cone_blue.dae", "cone_yellow.dae", or "cone_big.dae"
    # Note: large orange uses "cone_big.dae" regardless of color
```

**Critical plugins embedded in every generated world:**
```xml
<plugin filename="gz-sim-physics-system" .../>
<plugin filename="gz-sim-user-commands-system" .../>
<plugin filename="gz-sim-scene-broadcaster-system" .../>
<plugin filename="gz-sim-sensors-system" ...>   <!-- camera, LiDAR -->
  <render_engine>ogre2</render_engine>
</plugin>
<plugin filename="gz-sim-imu-system" .../>       <!-- IMU — SEPARATE from sensors -->
```

**Gotcha: `gz-sim-imu-system` is NOT included in `gz-sim-sensors-system`**. Camera and LiDAR publish through the sensors plugin. The IMU needs its own plugin. Without it, the IMU sensor entity exists in the ECS but never publishes. There is no error message — the entity just silently produces no data.

**Gotcha: mesh URIs** — `file:///absolute/path/to/cone_blue.dae`. These paths come from `resolve_track_mesh_dir()` which walks up from `__file__` to find the source tree. If you move the package, the absolute paths in old generated world files break. Generated worlds are meant to be ephemeral (deleted on stop), so this is fine.

---

### `simulator/fs_track_generator/artifacts.py`

**Problem it solves:** Knows where to put files and how to version them. No other module should hardcode paths.

**What breaks without it:** Files written to random locations; can't find them again; Gazebo can't load the world.

**Key functions:**

```python
def resolve_generated_dir(output_dir=None) -> Path:
    # Priority: explicit output_dir → source tree walk → ament_index
    # Default location: simulator/fs_track/worlds/generated/

def next_map_version(output_dir: Path) -> int:
    # Scans for map_layout_generated_N.world, returns N+1
    # Thread-unsafe: two concurrent launches will race on version numbers

def map_file_set(output_dir: Path, version: int) -> dict[str, Path]:
    # Returns: {"world": ..., "cones": ..., "meta": ..., "bridge": ..., "bridge_sensors": ...}
    # All files share the stem "map_layout_generated_{version}"

def write_structured_file(path: Path, payload: dict) -> None:
    # Writes JSON to a .yaml file (yes, JSON in a .yaml file)

def read_structured_file(path: Path) -> dict:
    # Reads JSON from a .yaml file

def default_maps_dir() -> Path:
    # Returns simulator/fs_track/worlds/maps/ — the user-provided custom maps dir
    # Uses the same source-tree walk as resolve_generated_dir

def list_map_files(maps_dir=None) -> list[Path]:
    # Returns sorted list of .world/.sdf files from the custom maps directory
    # Returns [] if the directory doesn't exist (non-fatal — TUI shows no custom maps)
```

**BIG GOTCHA: `.yaml` files contain JSON, not YAML.** The file extension is `.yaml` for convention, but the content is `json.dumps(...)`. Never try to load these with `yaml.safe_load()` (it works since JSON is valid YAML, but only if you use a proper YAML library). `read_structured_file` uses `json.loads` — don't swap it for a YAML loader.

**Path resolution logic** (in `_workspace_source_dir()`):
1. Walk up from `__file__` (the installed `.py` location)
2. Look for `fs_track/worlds/generated` relative to each parent
3. Works for both source layout (`src/simulator/...`) and installed layout (`install/simulator/...`)
4. Falls back to `ament_index` if the walk fails

---

### `simulator/fs_track_generator/ros_bridge.py`

**Problem it solves:** The joint_state Gazebo topic contains the world name and model name: `/world/fs_world_1/model/fs_car/joint_state`. This can't be static config — it changes every launch. This module generates the bridge YAML dynamically.

**What breaks without it:** `robot_state_publisher` gets no joint angles → can't publish transforms → RViz shows broken TF tree → car model collapses.

**Key insight:** Only the control bridge is dynamic. The sensors bridge is fully static (camera, LiDAR, scan topics don't embed world/model names). That's why `SENSORS_BRIDGE_CONFIG` is a module-level constant string.

```python
joint_state_topic = f"/world/{world_name}/model/{model_name}/joint_state"
```

This one line is why the bridge config must be regenerated per launch.

---

### `simulator/fs_track_generator/pipeline.py`

**Problem it solves:** The single function that calls everything else in the right order.

**What breaks without it:** You'd have to call generate → validate → write artifacts in the right sequence manually. Easy to forget a step.

```python
def generate_versioned_track(...) -> dict:
    dir = resolve_generated_dir(output_dir)
    version = next_map_version(dir)
    paths = map_file_set(dir, version)
    world_name = f"fs_world_{version}"

    layout = generate_track_layout(...)     # raises if invalid
    validation = validate_track_layout(...)  # second call, result saved to meta
    write_track_artifacts(...)              # writes all 5 files

    return {"version", "paths", "layout", "validation", "world_name", ...}
```

**Gotcha:** `generate_track_layout()` already validates internally and raises on failure. `validate_track_layout()` is called a second time here. The second result is only used for writing to the meta file — the tracks is already guaranteed valid at this point.

---

### `simulator/fs_track_generator/cli.py`

**Problem it solves:** Lets you run the track generator from the terminal without launching Gazebo. Useful for offline inspection.

```bash
ros2 run simulator generate_track --type trackdrive --seed 42 --track-length 450
```

**What breaks without it:** Nothing breaks in the main pipeline. This is a debugging/inspection tool.

---

### `simulator/fs_tui/sim_process.py`

**Problem it solves:** All the messy subprocess management that the UI shouldn't have to think about.

**What breaks without it:** `panel.py` would be unreadable and untestable.

**Key design choices:**

1. **`start_new_session=True`** — subprocess is a new process group leader. `os.killpg(pgid, signal.SIGTERM)` sends the signal to every process in the group (ros2 launch + all child nodes + Gazebo). Without this, `kill(pid, SIGTERM)` only kills the top-level `ros2` process and leaves Gazebo running.

2. **Two background tasks per launch:**
   - `_reader_task` — streams stdout to the log, colourizes `[sim]`, error, warn lines
   - `_watcher_task` — waits for the process to exit, updates UI state

3. **`_load_start_pose` with 5s delay** — called once via `set_timer(5.0, ...)`. Reads the `_meta.yaml` that the launch file just wrote. If not ready, retries every 3s. This is a polling strategy because we have no clean signal from the launch process that track generation is done.

4. **`_cleanup_generated_track`** — only deletes if `meta["delete_on_stop"] is True`. The launch file also registers an `OnShutdown` cleanup. They can both run; `FileNotFoundError` is silently ignored.

5. **GAZEBO_KILL_PATTERNS** — `pkill -f` each pattern on launch start, to kill any stale Gazebo from a previous crashed session. Two passes: SIGTERM then SIGKILL with 0.5s sleep between.

6. **Tool subprocess management** — `_tool_procs: dict[str, asyncio.subprocess.Process]` tracks named tool processes (RViz2, PS teleop). Calling `_launch_tool("rviz", cmd)` terminates any previous instance of that tool before starting a new one. `_kill_tool(name)` pops the entry and calls `terminate()` — used by `stop()` to close RViz2 when the simulation stops. All remaining tool procs are cleaned up in `close()` on TUI exit.

**Gotcha: `_watcher_task` race condition** — the `finally` block in `_watch_process` sets `self._launch_proc = None`. But `_terminate_launch_proc` also sets it to None before awaiting. The code snapshots `proc = self._launch_proc` and clears the attribute immediately to avoid racing with the watcher.

---

### `simulator/fs_tui/panel.py`

**Problem it solves:** The UI layer. Uses Textual (terminal UI library) for a clean dashboard.

**What breaks without it:** You'd have to use `ros2 launch` directly from the terminal.

**Key patterns:**

- `_on_radio_changed` handles both `render-radio` and `track-type-radio`. The track type handler also checks for `rb-map-N` IDs (custom maps) and sets `_custom_map_file` accordingly. When a built-in track type is selected, `_custom_map_file` is reset to `None`.
- `_on_button_pressed` uses a local `action_map` dict from button IDs to action method names, then dispatches sync or async based on `inspect.isawaitable`.
- **Custom maps panel**: `_map_files = list_map_files()` is populated in `__init__`. Custom map `RadioButton` widgets (`id="rb-map-0"`, etc.) are yielded inside the SAME `RadioSet` as built-in track types, giving automatic Textual mutual exclusion. If `maps/` is empty, no extra buttons appear.
- **Tool buttons**: RViz2 and PS Teleop are in a `Vertical` group in the config panel. They are always enabled — you can launch RViz2 before or after starting the simulation. Pressing **Stop All** also terminates any open RViz2 window.
- **`SimulationConfig`** — `weather` and `seed` removed. `map_file: str | None = None` added. When `map_file` is set, the launch file skips track generation entirely.
- `StatusBox` is a `Static` widget with a 1Hz internal timer. It self-updates elapsed time display.
- `_suppress_loop_closed_noise()` — filters `RuntimeError: Event loop is closed` from asyncio transport `__del__` firing after shutdown. This is a known Python bug with subprocess transports.

---

### `simulator/fs_bringup/launch/sim.launch.py`

**Problem it solves:** The user-facing top-level launch. Handles `track_type`, `render_mode`, `map_file`, etc. and wires the generator into the ROS2 launch graph.

**Key design:** `OpaqueFunction` runs `_generate_and_launch()` at launch time (not at launch-description parse time). This lets it call Python (the track generator) and then return `IncludeLaunchDescription` actions dynamically.

**Custom map bypass:** When `map_file` is non-empty, the `OpaqueFunction` returns immediately with a simple `IncludeLaunchDescription` — no track generation, no artifact files, no `OnShutdown` cleanup. The world name defaults to the file's stem. `bridge_config` and `sensors_bridge_config` fall back to the defaults in `spawn_vehicle.launch.py` (the static configs from `fs_vehicle`).

**`seed` and `weather` removed:** Hardcoded as `seed=0, weather="dry"` in the `generate_versioned_track()` call. These arguments no longer appear as `DeclareLaunchArgument` and are not user-facing. The `test_trackdrive` default length is now `300.0 m` (was `150.0 m`).

**What breaks without it:** You can still use `spawn_vehicle.launch.py` directly, but you'd have to generate the track manually first.

---

### `fs_vehicle/launch/spawn_vehicle.launch.py`

**Problem it solves:** All the Gazebo + URDF + bridge plumbing in one place.

**Startup sequence:**
1. `SetEnvironmentVariable` / `AppendEnvironmentVariable` — must run before any node starts
2. `gz_sim` — Gazebo loads the world
3. `robot_state_publisher` — publishes `/robot_description` from xacro
4. `spawn_car` — delayed 0.1s, reads `/robot_description`, spawns entity into Gazebo
5. `bridge` + `sensors_bridge` — two separate `ros_gz_bridge` nodes
6. `rviz` — delayed 6s (needs TF tree to be populated first)

**`GZ_SIM_RESOURCE_PATH`** — points to the **parent** of the `fs_vehicle` share directory. Gazebo resolves `model://fs_vehicle/meshes/cone_blue.dae` by looking for a folder named `fs_vehicle` inside any directory in this path.

**`GZ_SIM_SYSTEM_PLUGIN_PATH`** — points to the lib directory where `libFsAckermannDrive.so` lives.

---

### `fs_vehicle_plugins/src/fs_ackermann_drive.cpp`

**Problem it solves:** The default `gz-sim-ackermann-steering-system` doesn't exist in Jazzy/Harmonic. This is a custom plugin that converts `cmd_vel` Twist messages into wheel velocities and steering angles.

**What breaks without it:** The car has no drive. It just sits.

**How it works (every physics tick at 125 Hz):**

1. **Speed ramping** — `current_speed_` approaches `target_speed_` at `max_accel=4.0 m/s²` or `max_decel=8.0 m/s²`. Prevents instant velocity changes that make teleop jerky.
2. **Rear wheel drive** — converts `current_speed_` to angular velocity (`v / wheel_radius`) and writes to both rear `JointVelocityCmd` components.
3. **Ackermann geometry** — given centre steer angle δ:
   - `R = wheel_base / tan(δ)` (turning radius)
   - `δ_left = atan(L / (R - w/2))`, `δ_right = atan(L / (R + w/2))`
   - Inner wheel turns tighter than outer → no tyre scrub
4. **P-controller for steering** — reads current hinge angles from `JointPosition` component, computes error, writes proportional `JointVelocityCmd`. Gain `steer_kp=8.0`, rate limit `max_steer_rate=3.0 rad/s`.

**Thread safety:** `OnCmdVel` runs on a Gazebo-transport thread. `PreUpdate` runs on the physics thread. `mutex_` protects `target_speed_` and `target_steer_`.

**Gotcha — low-speed steering:** At `|v| < 0.05 m/s`, Ackermann inverse kinematics divides by `|v|` → infinity. The fallback maps `angular.z` directly to a steering angle: `steer = w * (wheel_base / 2.0)`. This feels slightly different from high-speed behaviour but is good enough for low-speed manoeuvring.

---

### `fs_vehicle/worlds/empty_track.sdf`

**Problem it solves:** The static world used when launching `spawn_vehicle.launch.py` directly (no generated track).

**Must contain** the same set of plugins as generated worlds, including `gz-sim-imu-system`. If you add a plugin to one, add it to the other.

---

## Key Concepts & Patterns Used

### Hardcoded FSG circuit templates

```
Fixed normalized polygon (e.g. 14 points for trackdrive)
→ Centripetal Catmull-Rom (α=0.5) → smooth closed curve
→ Translate so curve[0] == (0, 0)
→ Scale uniformly to target_length_m
→ Resample at 1.25m spacing
```

**Trackdrive template (~450 m):** D-shaped FSG-style layout. Main straight heading east → sweeping right hairpin → S-curve section → left sweep → final approach back to start. Minimum turn radius ≈ 4.3m at 450m scale, ≈ 5.7m at 650m scale.

**Test trackdrive template (~300 m):** Kidney-shaped compact circuit. Main straight → right hairpin → back straight → left sweep → return. Minimum turn radius ≈ 8.8m at 300m scale (wider corners relative to track length than the full trackdrive).

Both templates are scale-invariant: passing `target_length_m=650.0` produces the same shape, just larger. The car always starts at the origin heading east.

Why Catmull-Rom over Bezier? Catmull-Rom passes through all control points (interpolating), so the track shape directly reflects the control points. Bezier only approximates them.

Why **centripetal** Catmull-Rom? Uniform parameterisation (α=0) creates curvature spikes when short segments sit next to long ones — e.g., a 0.07-unit return stub next to a 1.1-unit main straight segment. At 450m scale, this produced a ≈0.65m circumradius spike at the start/finish gate. Centripetal (α=0.5) parameterises by √(chord_length), giving proportional weight to the tangent at each control point and eliminating the spike.

### Closed-loop geometry invariant

The centerline's last point equals the first: `centerline[-1] == centerline[0]`. This means `boundary[-1] == boundary[0]`. Any code that iterates the boundary must account for this if it processes the final segment.

### Adaptive cone spacing (curvature-based)

```python
curvature = min(_local_curvature(points, index), 0.25)
blend = curvature / 0.25                 # 0 = straight, 1 = corner
spacing = 5.0 + (2.5 - 5.0) * blend     # 5.0m straight → 2.5m corner
```

FSG 2024 max: 5m on straights, 3m on corners. We use 2.5m corners (3m failed overlap validation in tight hairpins).

### The two-bridge split

`ros_gz_bridge` is single-threaded per node. A 32-channel LiDAR at 10 Hz generates ~23,000 points per scan. Serializing that blocks the thread. If cmd_vel uses the same bridge node, steering commands get delayed during sensor updates.

Solution: two bridge nodes = two OS processes = two threads:
- `ros_gz_bridge_control`: cmd_vel, odom, tf, joints, imu, clock (all small messages)
- `ros_gz_bridge_sensors`: scan, pointcloud, camera (all large messages)

### Versioned ephemeral artifacts

Every launch generates a new version number (scan for highest `map_layout_generated_N.world`). `delete_on_stop=True` ensures these are cleaned up. This prevents stale world files from accumulating and confusing the "latest" resolution logic.

### GameMode integration

```python
# In SimController.launch():
_gamemoderun = shutil.which("gamemoderun") or "/usr/games/gamemoderun"
_use_gamemode = Path(_gamemoderun).exists()
cmd = ([_gamemoderun] if _use_gamemode else []) + ["ros2", "launch", ...]

# In spawn_vehicle.launch.py:
AppendEnvironmentVariable('LD_PRELOAD', '/usr/lib/x86_64-linux-gnu/libgamemodeauto.so.0')
```

GameMode sets CPU governor to `performance` and pins processes to P-cores. The `LD_PRELOAD` approach applies it to all child processes automatically. The `gamemoderun` prefix in the TUI applies it to the whole launch tree.

---

## How Core Features Work

### Custom map pipeline

1. `FsSimApp.__init__()` calls `_load_map_files()` → `list_map_files()` → scans `simulator/fs_track/worlds/maps/` for `.world`/`.sdf` files.
2. In `compose()`, for each file a `RadioButton(f"Map: {path.name}", id=f"rb-map-{i}")` is yielded inside the **same `RadioSet`** as the built-in track types.
3. When the user selects one, `_on_radio_changed` sets `self._custom_map_file = str(path)`.
4. `action_launch()` passes `map_file=self._custom_map_file` to `SimulationConfig`.
5. `SimController.launch()` appends `map_file:=…` to the `ros2 launch` command.
6. `sim.launch.py` detects non-empty `map_file`, skips generation, launches Gazebo directly.

To add a custom map: drop a `.world` or `.sdf` file into `simulator/fs_track/worlds/maps/` and restart the TUI.

### Tool launchers (RViz2, PS Teleop, Keyboard Teleop)

```python
# In sim_process.py:
async def launch_rviz(rviz_config=None):
    cmd = ["ros2", "run", "rviz2", "rviz2"]
    if rviz_config: cmd += ["-d", rviz_config]
    await _launch_tool("rviz", cmd)

async def launch_ps_teleop():
    await _launch_tool("ps_teleop", ["ros2", "launch", "fs_vehicle", "ps_teleop.launch.py"])

async def launch_keyboard_teleop():
    terminal = _find_terminal_cmd()  # xterm / gnome-terminal / etc.
    kb_cmd = ["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard"]
    await _launch_tool("kb_teleop", (terminal + kb_cmd) if terminal else kb_cmd)
```

`_launch_tool(name, cmd)` terminates any previous instance of that tool, then spawns the new one as an independent subprocess (`start_new_session=True`). Tool processes are stored in `_tool_procs` and cleaned up in `close()`.

The RViz2 config is resolved by `_find_rviz_config()` in `panel.py`: first tries `ament_index` → `get_package_share_directory("fs_vehicle")/config/fs_car.rviz`, then walks up the source tree as fallback.

### Reset Car

1. TUI calls `SimController.reset_car()`
2. Controller reads `_start_pose` from loaded metadata
3. Publishes zero Twist to `/cmd_vel` (stops the car)
4. Calls `gz service -s /world/{world_name}/set_pose` with a Pose proto string
5. Gazebo teleports the model

The Pose proto string is hand-built as text because there's no Python Gazebo-proto library installed:
```
name: "fs_car" position: {x: 1.23 y: 4.56 z: 0.5} orientation: {w: 0.999 x: 0.0 y: 0.0 z: 0.035}
```
The orientation quaternion is computed from yaw only (`w = cos(yaw/2)`, `z_rot = sin(yaw/2)`).

### Skidpad geometry (FSG D4.1)

```
Right circle: centre (0, +9.125m), inner R=7.625m, outer R=10.625m
Left circle:  centre (0, -9.125m), same radii
16 inner orange cones / circle, 13 outer blue(right)/yellow(left) / circle
Crossing corridor exclusion: |y| < 2.2m AND |x| < 6.5m → no cones placed there
Timekeeping gate: 2 large orange at (0, ±1.9m)
```

### Cone placement in skidpad vs trackdrive

**Trackdrive/test_trackdrive:** Cones are placed by walking along the boundary at adaptive spacing. `_place_side_cones` is a distance-accumulator algorithm (not index-based).

**Skidpad/acceleration:** Cones are placed at fixed angular or linear intervals, hardcoded in `_generate_skidpad_layout()` and `_generate_acceleration_layout()`. No `_place_side_cones` call.

---

## Danger Zones & Gotchas

### 1. `generate_track_layout()` raises on bad geometry

```python
validation = validate_track_layout(layout, min_turn_radius_m=min_turn_radius_m)
if not validation["valid"]:
    raise ValueError(f"Generated track failed validation: {validation}")
```

Since the templates are hardcoded and validated, this should never fire in normal use. It would only trigger if you: (a) pass a `target_length_m` that forces the spline to collapse (e.g., 0.0), or (b) modify the template control points to create a self-intersecting layout.

### 2. `min_turn_radius_m` default

`generate_track_layout()` defaults to `4.0 m`. `sim.launch.py` also passes `4.0 m`. These match. The FSG-standard trackdrive template achieves ≈4.3m at 450m scale — tight but valid. If you reduce `target_length_m` significantly (< ~300m for trackdrive), the scaled template may produce tighter corners.

### 3. Stale Gazebo on TUI re-launch

If Gazebo crashed without cleanup, `_kill_stray_gazebo()` runs `pkill -f` on 7 patterns at launch start. This uses `SIGTERM` then `SIGKILL` with 0.5s between. If pkill hangs for > 2s (`PKILL_TIMEOUT_S`), it continues anyway. In the worst case a stale Gazebo is still running when the new world loads → port conflict → launch fails silently.

### 4. Race on `next_map_version()`

`next_map_version` scans the directory for the highest version number. If two launches run simultaneously (e.g., a test suite and a manual launch), they both read the same "latest" and try to write the same version. The second write silently overwrites the first. Single-user usage is fine; CI with parallel test workers would race.

### 5. `.yaml` files are JSON

`write_structured_file` calls `json.dumps()` and writes to a `.yaml` file. `read_structured_file` calls `json.loads()`. This works because JSON is valid YAML, but if you ever load these with a strict YAML-only parser, you'll get confusing errors.

### 6. `boundary[-1] == boundary[0]` causes cone overlap

The Catmull-Rom function closes the loop by appending `points[0]` at the end. `_resample_polyline` also closes the loop. `_build_boundaries` produces `boundary[-1] == boundary[0]`. The last segment in `_place_side_cones` ends at the start position, potentially placing a cone right where the start gate orange cone lives. **Fixed** by the 0.2m dedup filter, but if you modify cone placement, re-check overlap validation.

### 7. IMU sensor vs IMU system plugin

The IMU sensor in the car URDF creates a Gazebo sensor entity. But an entity is just data — it needs a **system** to drive it. `gz-sim-sensors-system` handles camera and LiDAR. The IMU needs `gz-sim-imu-system` loaded separately. Without it:
- `gz entity -v 4` shows `Sensor [803]` (entity exists)
- `gz topic -l` shows `/imu/data` (topic registered)
- `gz topic --list-publishers /imu/data` → `No publishers` (nobody driving it)
- ROS2 `/imu/data` is completely silent

### 8. `start_new_session=True` and the kill chain

The launch subprocess uses `start_new_session=True`, making it a new process group leader. `SimController._terminate_launch_proc()` kills the group with `os.killpg(pgid, signal.SIGTERM)`. This is the correct way to kill a ros2 launch tree (it has multiple child processes). Without `start_new_session=True`, `os.getpgid()` returns the TUI's own pgid and you'd SIGTERM the TUI itself.

### 9. The 6-second RViz delay

RViz starts 6 seconds after the world loads. This is intentional: the TF tree isn't populated until after Gazebo spawns the car, the bridge publishes joint states, and `robot_state_publisher` starts transforming them. Starting RViz earlier causes "frame not found" errors that persist even after the TF tree is ready.

### 10. `LD_PRELOAD` and libgamemodeauto

`spawn_vehicle.launch.py` appends to `LD_PRELOAD` unconditionally. If `libgamemodeauto.so.0` doesn't exist on the machine, Gazebo will fail to start with a dynamic linker error. The path is hardcoded to `/usr/lib/x86_64-linux-gnu/libgamemodeauto.so.0`. On ARM or non-Ubuntu systems this path will be different.

---

## How to Debug Common Issues

### IMU data not publishing (`/imu/data` silent)

```bash
# Check if topic exists at all
gz topic -l | grep imu

# Check for publishers
gz topic --list-publishers /imu/data
# "No publishers" → gz-sim-imu-system not loaded

# Verify plugin is in world
grep -r "gz-sim-imu-system" src/
# Should appear in both empty_track.sdf and world.py
```

### Car spawns but doesn't move

```bash
# Check if FsAckermannDrive loaded
gz topic -l | grep cmd_vel  # should exist in Gazebo topics

# Check if bridge is running
ros2 node list | grep bridge

# Check if cmd_vel reaching Gazebo
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
gz topic -e -t /cmd_vel  # should echo the message
```

### TF tree broken / RViz shows collapsed model

```bash
ros2 topic echo /joint_states  # should show joint angles changing
# If empty → bridge control config has wrong joint_state topic name
# The topic must be: /world/fs_world_N/model/fs_car/joint_state

# Check the generated bridge config
cat src/simulator/fs_track/worlds/generated/map_layout_generated_*_bridge.yaml | grep joint_state
```

### Track generation raises `ValueError`

```bash
# Test a seed offline
python3 -c "
from fs_track_generator.layout import generate_track_layout
from fs_track_generator.validation import validate_track_layout
layout = generate_track_layout(track_type='trackdrive', seed=YOUR_SEED,
    target_length_m=450.0, min_turn_radius_m=4.0)
print(validate_track_layout(layout))
"
```

### Cone overlaps failing in tests

```python
# Diagnose which cones overlap
from fs_track_generator.layout import (
    _load_circuit_centerline, _TRACKDRIVE_TEMPLATE,
    _build_boundaries, _place_cones, _start_finish_cones
)
import math

cl = _load_circuit_centerline(_TRACKDRIVE_TEMPLATE, 450.0)
lb, rb = _build_boundaries(cl)
bcones = _place_cones(cl, lb, rb, "trackdrive")
sfcones = _start_finish_cones(cl, lb, rb)
all_cones = bcones + sfcones

def dist(a, b): return math.hypot(a[0]-b[0], a[1]-b[1])
for i, c1 in enumerate(all_cones):
    for j, c2 in enumerate(all_cones[i+1:], i+1):
        if dist((c1.x,c1.y),(c2.x,c2.y)) < 0.2:
            print(f"[{i}]{c1.color},{c1.size} vs [{j}]{c2.color},{c2.size}")
```

### Stale Gazebo after crash

```bash
# Manual cleanup
pkill -f "gz sim"
pkill -f "gz_sim_vendor"
pkill -f "ruby.*gz"
pkill -f "ros2 launch simulator"
sleep 1
pkill -9 -f "gz sim"  # nuclear option
```

### `Event loop is closed` noise on TUI exit

This is suppressed by `_suppress_loop_closed_noise()` in `panel.py`. If you see it, the asyncio transport cleanup in `SimController.close()` didn't run. Check that `on_unmount` calls `await self._controller.close()`.

---

## Things That Are Easy to Break

### Adding a new track type

You need to touch **every** one of these:
- `layout.py` — add elif branch, implement geometry
- `validation.py` — add to conditional skips if it's open or has unusual geometry
- `cli.py` — add to `choices=` in argparse
- `sim.launch.py` — add to `_DEFAULT_LENGTHS` dict and description string
- `panel.py` — add `RadioButton` in `compose()` and entry in `TRACK_TYPE_BY_RADIO_ID`
- `test_track_generator.py` — add test

Missing any one of these causes either a crash (invalid type), silent wrong behavior, or missing UI option.

### Changing the world naming convention

Generated worlds are named `fs_world_{version}`. This name is embedded in:
- The world SDF: `<world name="fs_world_1">`
- The bridge config: `/world/fs_world_1/model/fs_car/joint_state`
- The metadata: `world_name: "fs_world_1"`
- The spawn arguments passed from `sim.launch.py` to `spawn_vehicle.launch.py`

If you change the naming in one place but not all, joint_states won't bridge and the TF tree breaks.

### Modifying cone placement order

`spacing_rule_ok` in the validator checks **consecutive** cones in list order. `_place_cones` returns blue cones first, then yellow. If you change the order (e.g., interleave them), the spacing check will compare a blue cone to a yellow cone and potentially find a 0.5m "spacing violation" between cones on opposite sides of the track.

### Changing `_resample_polyline` to not close the loop

The closed loop (`centerline[-1] == centerline[0]`) is assumed by:
- `closed_loop_ok` check in validator (`< 2.0m` between first and last)
- `_build_boundaries` — the last boundary point = first boundary point
- `_place_side_cones` — the last segment ends at the start position

Removing the `resampled.append(resampled[0])` line makes the track "open" and breaks the closed-loop check AND changes where the last cones land.

### Moving cone mesh files

`resolve_track_mesh_dir()` walks up from `__file__` to find `fs_track/meshes`. The DAE mesh `file://` URIs in high-render-mode SDF are absolute paths resolved at generation time. If you move the meshes after generating worlds, old world files have broken URIs. Since worlds are ephemeral (`delete_on_stop=True`), this only matters if you use `view_generated_track.launch.py` or disable cleanup.

---

## Questions to Test Your Own Understanding

**1. Why does the bridge config need to be regenerated for every launch?**

Because the Gazebo joint_state topic includes the world name: `/world/fs_world_1/model/fs_car/joint_state`. Every launch gets a new world version number, making the topic name different. The control bridge YAML must be written with the correct topic each time, otherwise `robot_state_publisher` gets no joint angles and the TF tree breaks.

**2. What happens if you call `generate_track_layout()` without passing `min_turn_radius_m`?**

The default is now `4.0 m`, aligned with `sim.launch.py`. Since the templates are hardcoded (not seeded-random), there's no seed-sensitivity issue. Any valid `target_length_m` ≥ ~300m will produce a track that passes the 4.0m threshold.

**3. How does the TUI know the world name and start pose after launching?**

It reads `_meta.yaml` (via `_read_latest_meta()`) 5 seconds after launch starts. The meta file is written by the track generator inside the `OpaqueFunction` in `sim.launch.py` before Gazebo starts. If the 5s timer fires before the file is written (unlikely but possible on slow machines), the controller retries every 3 seconds.

**4. The generator runs `validate_track_layout()` inside `generate_track_layout()`. Then `generate_versioned_track()` calls it again. Why the second call?**

The first call in `generate_track_layout()` raises on failure — it's a correctness gate. The second call in `generate_versioned_track()` is for **recording** — the validation report (including metrics like `min_turn_radius_m` and `cone_count`) is written to the meta YAML for debugging. You couldn't easily get it from the first call since it's internal to the generator.

**5. Why is `start_new_session=True` critical in `SimController.launch()`?**

`ros2 launch` spawns many child processes (Gazebo, RSP, bridges, etc.). Using `os.killpg(pgid, signal.SIGTERM)` sends the signal to the entire process group. Without `start_new_session=True`, the subprocess joins the TUI's process group, and killing the group would SIGTERM the TUI itself.

**6. What are the two Gazebo system plugins that handle sensors, and why are they separate?**

`gz-sim-sensors-system` handles camera and LiDAR. `gz-sim-imu-system` handles the IMU. They are implemented as separate `.so` libraries and are loaded separately. The common mistake is assuming `gz-sim-sensors-system` handles all sensor types — it doesn't handle IMU. Without `gz-sim-imu-system`, the IMU sensor entity exists in the ECS but produces no data, silently.

**7. The `.yaml` artifact files are read with `json.loads()`. Why not `yaml.safe_load()`?**

By design: `write_structured_file` writes `json.dumps(...)` to avoid adding a `pyyaml` dependency in the generator (which must run without full ROS sourced). JSON is valid YAML, so a YAML parser would also work, but `json` is stdlib. Don't swap the reader to `yaml.safe_load` — it works, but it would silently diverge if someone ever writes actual YAML to these files.

**8. What causes cone overlap validation to fail, and how is it fixed?**

The `_start_finish_cones()` function places a gate cone at 5m along the centerline on the left boundary. `_place_side_cones()` also places a cone at that exact same position (it's the first boundary cone placed, at one spacing interval from the start). The fix: boundary cones within 0.2m of any start/finish cone are filtered out before combining the cone lists. The spacing validator doesn't check the wrap-around gap (last → first cone), so removing a boundary cone near the start doesn't break spacing validation.

**9. How does the Ackermann plugin handle the steering angle at very low speed?**

At `|v| < 0.05 m/s`, the Ackermann formula (`steer = atan(L * ω / |v|)`) would divide by near-zero. The plugin falls back to `steer = clamp(ω * (wheel_base / 2.0), ±max_steer)`. This is a linear approximation that feels slightly different from normal Ackermann but is fine for parking-speed manoeuvring.

**10. What's wrong with `render_mode="performance"` before OPT-013, and how was it fixed?**

The `shadows` variable in `world.py` was hardcoded to `"true"` and never updated based on `render_mode`. So even in performance mode, the `<shadows>true</shadows>` and `<cast_shadows>true</cast_shadows>` tags were still in the world, casting shadows for all 180+ cones. The fix: `shadows = "false" if render_mode == "performance" else "true"`.

---

## How to Extend This Project

### Add a new track type (e.g., "endurance")

1. Add `_ENDURANCE_TEMPLATE: list[tuple[float, float]] = [...]` in `layout.py`
2. Add `elif track_type == "endurance":` branch in `generate_track_layout()` that calls `_load_circuit_centerline(_ENDURANCE_TEMPLATE, target_length_m)`
3. Add `"endurance": 1000.0` to `_DEFAULT_LENGTHS` in `sim.launch.py`
4. Add to `choices=` in `cli.py`
5. Add radio button in `panel.py` `compose()` and entry in `TRACK_TYPE_BY_RADIO_ID`
6. Add test in `test_track_generator.py`
7. Verify min radius ≥ 4.0m across the range of `target_length_m` you intend to support

### Add a new sensor

1. Add sensor block to `fs_vehicle/urdf/sensors.urdf.xacro`
2. Add Gazebo topic → ROS2 topic mapping in `ros_bridge.py` (decide: control or sensors bridge)
3. If it needs a new Gazebo system plugin (like IMU did), add it to both `empty_track.sdf` and the generated world header in `world.py`
4. Add to `ros_gz_bridge_sensors.yaml` if static (no world/model name in topic)

### Change physics rate

Two files must stay in sync:
- `world.py`: `max_step_size` and `real_time_update_rate` in the `<physics>` block
- `empty_track.sdf`: same two values in the same block

The dt value flows into `FsAckermannDrive::PreUpdate` via `info.dt` — the plugin adapts automatically.

### Add ROS2 services (e.g., lap timer)

Add a node to `spawn_vehicle.launch.py`. Add any new Gazebo ↔ ROS2 service bridges to the control bridge config in `ros_bridge.py`.

---

## Notes for Future-Me

- **The generated world files are ephemeral.** They live in `simulator/fs_track/worlds/generated/` and are deleted when the sim stops (`delete_on_stop=True` is hardcoded in the launch). Don't reference them by path outside of a running launch session.

- **`min_turn_radius_m=4.0` is the magic number.** The launch file, the tests, and the TUI all use 4.0, and `generate_track_layout()` now defaults to 4.0 as well.

- **The TUI controller is fully decoupled from Textual.** `SimController` takes callbacks (`log`, `schedule_timer`, `on_launch_started`, etc.) as constructor arguments. You could test it without a running TUI by passing mock callbacks. This was deliberate — the UI and the subprocess management are separate concerns.

- **The skidpad geometry follows FSG D4.1 exactly** (as of OPT-012). If FSG updates the rules, check `_SKIDPAD_HALF_SPAN`, `_SKIDPAD_INNER_R`, `_SKIDPAD_OUTER_R`, `_SKIDPAD_N_INNER`, `_SKIDPAD_N_OUTER`. The corridor exclusion zone constants (`_SKIDPAD_CORRIDOR_HALF_Y`, `_SKIDPAD_CORRIDOR_X_LIMIT`) were derived by calculating where outer boundary cones land — if you change the radii, recalculate these.

- **`test_trackdrive` and `trackdrive` use different templates.** `test_trackdrive` uses a kidney-shaped 12-point template at `target_length_m=300.0 m`; `trackdrive` uses a D-shaped 14-point FSG-style template at `450.0 m`. They are geometrically distinct — different corner counts, different aspect ratios.

- **Cone noise (`cone_noise` field in TrackLayout) is not implemented.** It's a placeholder for future work. The field exists in the dataclass and is written to meta, but the generator ignores it.

- **GameMode is optional but impactful.** The TUI checks for `gamemoderun` before adding it to the launch command. The launch file sets `LD_PRELOAD` unconditionally. If `gamemoded` isn't running, the preload does nothing (the library calls `gamemodeauto` which is a no-op if the daemon isn't there). The RTF difference between with/without gamemode on the test machine was ~10–15%.

- **The CHANGELOG is authoritative.** Every physics tuning decision and its outcome is documented in `CHANGELOG.md`. Before changing any physics parameter or track template, read the relevant OPT entry to understand what was already tried and why it was changed.

- **Track templates are the source of truth for circuit shape.** `_TRACKDRIVE_TEMPLATE` and `_TEST_TRACKDRIVE_TEMPLATE` in `layout.py` are the only things that determine track geometry. Tweaking a control point will affect the layout at ALL scales. Always verify min_radius ≥ 4m after template changes (run `test_track_generator.py`).

- **Custom maps bypass ALL validation.** When `map_file` is set, the generator is skipped entirely — no validation, no metadata, no `delete_on_stop`. The TUI won't display track info or cone count. Reset Car won't work (no `_start_pose` — it's read from the meta file). For custom maps, reset the car manually via `gz service`.

- **Tool subprocesses outlive the simulation.** RViz2, PS teleop, and keyboard teleop processes are stored in `_tool_procs` and are NOT killed when the simulation stops (only when the TUI exits). This is intentional — you may want to keep RViz open after stopping a run.

- **`view_generated_track.launch.py` exists** for launching an already-generated world directly, including the saved car start pose and bridge config.
