# simulator

`simulator` owns track generation, generated-map artifacts, launch helpers, and the Textual TUI for the Formula Student simulator.

It does not own the active vehicle runtime by itself. The car model, spawn flow, and Gazebo plugins still come from [`fs_vehicle`](/home/rohith/Fs_ws/src/fs_vehicle) and [`fs_vehicle_plugins`](/home/rohith/Fs_ws/src/fs_vehicle_plugins).

## What this package does

- generates Formula Student layouts and validates them
- writes Gazebo world files plus cone, metadata, and bridge sidecars
- launches either a freshly generated world or an existing generated world
- provides a TUI for common launch and tooling workflows

## Package layout

```text
simulator/
├── fs_bringup/
│   └── launch/
│       ├── sim.launch.py
│       └── view_generated_track.launch.py
├── fs_track/
│   ├── meshes/
│   └── worlds/
│       ├── generated/
│       └── maps/
├── fs_track_generator/
│   ├── artifacts.py
│   ├── cli.py
│   ├── layout.py
│   ├── models.py
│   ├── pipeline.py
│   ├── ros_bridge.py
│   ├── validation.py
│   └── world.py
├── fs_tui/
│   ├── panel.py
│   └── sim_process.py
└── test/
```

## Build

Build from the workspace root:

```bash
cd ~/Fs_ws
colcon build --packages-select simulator fs_vehicle fs_vehicle_plugins
source install/setup.bash
```

The ROS package name is `simulator`, all lowercase.

## Quick start

Generate a track:

```bash
ros2 run simulator generate_track --seed 42 --type trackdrive
```

Launch the latest generated track:

```bash
ros2 launch simulator view_generated_track.launch.py
```

Generate and launch in one step:

```bash
ros2 launch simulator sim.launch.py
```

Launch the TUI:

```bash
ros2 run simulator fs_tui
```

## Track generator CLI

The generator writes a new numbered artifact set on every run.

```bash
ros2 run simulator generate_track \
  --seed 42 \
  --type trackdrive \
  --track-length 450 \
  --min-turn-radius 4.0 \
  --cone-noise low \
  --weather dry
```

Supported track types:

- `trackdrive`
- `test_trackdrive`
- `skidpad`
- `acceleration`

Notes:

- `--track-length` applies to `trackdrive` and `test_trackdrive`
- `skidpad` and `acceleration` use fixed geometry
- `--cone-noise` is currently metadata only

## Launch files

### `sim.launch.py`

Generates a new track in-process, writes the artifacts, then launches Gazebo and spawns the car.

Examples:

```bash
ros2 launch simulator sim.launch.py
ros2 launch simulator sim.launch.py track_type:=test_trackdrive
ros2 launch simulator sim.launch.py track_type:=skidpad
ros2 launch simulator sim.launch.py track_type:=acceleration gz_headless:=true
ros2 launch simulator sim.launch.py render_mode:=performance
```

You can also bypass generation and launch a custom world directly:

```bash
ros2 launch simulator sim.launch.py map_file:=/absolute/path/to/custom.world
```

### `view_generated_track.launch.py`

Launches an already-generated world, reads its saved metadata, and spawns the car at the stored start pose.

```bash
ros2 launch simulator view_generated_track.launch.py
ros2 launch simulator view_generated_track.launch.py version:=3
ros2 launch simulator view_generated_track.launch.py gz_headless:=true
```

## TUI

Run it with:

```bash
ros2 run simulator fs_tui
```

The TUI can:

- choose a built-in track type
- choose render mode (`high` or `performance`)
- launch a custom map from `fs_track/worlds/maps/`
- launch and stop the simulator
- reset the car to the generated start pose
- launch RViz2, PS teleop, and keyboard teleop helpers

Custom maps are discovered from:

```text
/home/rohith/Fs_ws/src/simulator/fs_track/worlds/maps
```

If `textual` is missing, install it into the Python environment used by your ROS setup.

## Generated files

Generated maps live in:

```text
/home/rohith/Fs_ws/src/simulator/fs_track/worlds/generated
```

Each version creates:

- `map_layout_generated_N.world`
- `map_layout_generated_N_cones.yaml`
- `map_layout_generated_N_meta.yaml`
- `map_layout_generated_N_bridge.yaml`
- `map_layout_generated_N_bridge_sensors.yaml`

The sidecar files use `.yaml` filenames but are written as JSON-compatible structured text so they can be read back without extra parser dependencies.

## Teleop

After the simulator is running:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The active control topic is `/cmd_vel`.

## Developer notes

- [`pipeline.py`](/home/rohith/Fs_ws/src/simulator/fs_track_generator/pipeline.py) is the shared path for generate → validate → artifact write
- [`sim.launch.py`](/home/rohith/Fs_ws/src/simulator/fs_bringup/launch/sim.launch.py) and [`cli.py`](/home/rohith/Fs_ws/src/simulator/fs_track_generator/cli.py) both use that shared pipeline
- track geometry lives in [`layout.py`](/home/rohith/Fs_ws/src/simulator/fs_track_generator/layout.py)
- Gazebo world assembly lives in [`world.py`](/home/rohith/Fs_ws/src/simulator/fs_track_generator/world.py)
- generator tests live in [`test_track_generator.py`](/home/rohith/Fs_ws/src/simulator/test/test_track_generator.py)

## Troubleshooting

### `Package 'simulator' not found`

Source the workspace:

```bash
cd ~/Fs_ws
source install/setup.bash
```

### `ros2 run simulator fs_tui` fails with a Textual import or widget error

The TUI runs on the Python environment backing your ROS installation. Install `textual` there.

### The launched track is not the one you expected

`view_generated_track.launch.py` launches the highest numbered generated version unless you pass `version:=N`.

### The car spawns but an old Gazebo window is still around

Close stale Gazebo sessions before starting a new run. The TUI does best-effort cleanup, but manual launches can still leave leftovers behind.

## Verification

At the source-tree level, the generator tests pass:

```bash
python3 -m pytest /home/rohith/Fs_ws/src/simulator/test/test_track_generator.py -q
```
