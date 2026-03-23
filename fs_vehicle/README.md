# fs_vehicle

`fs_vehicle` contains the Formula Student car description, Gazebo launch file, bridge config, and world assets for the base simulator.

## Build

```bash
cd /home/rohith/Fs_ws
colcon build --packages-select fs_vehicle fs_vehicle_plugins
source install/setup.bash
```

`fs_vehicle_plugins` must be built too, because the car URDF loads the `FsAckermannDrive` Gazebo plugin from that package.

## Launch the car in the default world

```bash
ros2 launch fs_vehicle spawn_vehicle.launch.py
```

This launch file:
- starts Gazebo Sim
- starts `robot_state_publisher`
- spawns the `fs_car` model
- starts `ros_gz_bridge`

## Launch headless

```bash
ros2 launch fs_vehicle spawn_vehicle.launch.py gz_headless:=true
```

## Launch the car in a custom world

```bash
ros2 launch fs_vehicle spawn_vehicle.launch.py \
  world:=/absolute/path/to/world.sdf
```

## Spawn the car at a custom pose

```bash
ros2 launch fs_vehicle spawn_vehicle.launch.py \
  spawn_x:=0.0 \
  spawn_y:=0.0 \
  spawn_z:=0.5 \
  spawn_yaw:=0.0
```

## Useful notes

- The bridge config lives in `config/ros_gz_bridge.yaml`.
- The main launch file is `launch/spawn_vehicle.launch.py`.
- If you restart Gazebo while RViz is still open, RViz can warn about `TF_OLD_DATA`. Restart RViz after a sim reset to avoid that.
