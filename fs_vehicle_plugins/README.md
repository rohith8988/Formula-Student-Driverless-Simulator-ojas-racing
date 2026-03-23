# fs_vehicle_plugins

`fs_vehicle_plugins` builds the custom Gazebo system plugin used by the simulator drivetrain.

## What this package does

This package builds:

- `FsAckermannDrive`

That plugin is loaded by the car URDF from `fs_vehicle`. It is not launched directly with `ros2 launch`.

## Build

```bash
cd /home/rohith/Fs_ws
colcon build --packages-select fs_vehicle_plugins fs_vehicle
source install/setup.bash
```

## How to use it

Launch the car through `fs_vehicle`:

```bash
ros2 launch fs_vehicle spawn_vehicle.launch.py
```

Or launch the generated-track flow through `Simulator`:

```bash
ros2 run Simulator generate_track --seed 42 --type autocross
ros2 launch Simulator view_generated_track.launch.py
```

## How to confirm it is being used

- Gazebo should load the car without plugin path errors.
- The plugin library is installed under `install/fs_vehicle_plugins/lib`.
- The car will respond to `/cmd_vel` through the custom plugin path.

## Important note

There is no standalone launch file in this package right now. This package exists to provide the shared library that Gazebo loads when the vehicle model is spawned.
