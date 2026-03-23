"""Unified FS simulator launch.

Generates a track (in-process) then launches Gazebo + car in a single command.
The generated world file and metadata are written to the standard generated-maps
directory so the TUI and other tools can pick them up.

Usage
-----
  ros2 launch simulator sim.launch.py
  ros2 launch simulator sim.launch.py track_type:=skidpad
  ros2 launch simulator sim.launch.py track_type:=acceleration
  ros2 launch simulator sim.launch.py gz_headless:=true
  ros2 launch simulator sim.launch.py launch_rviz:=false
  ros2 launch simulator sim.launch.py map_file:=/path/to/custom.world
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Default target lengths per track type (metres).  Skidpad and acceleration
# have fixed geometries; the length argument is ignored for those two.
_DEFAULT_LENGTHS: dict[str, float] = {
    "trackdrive":      450.0,
    "test_trackdrive": 300.0,
    "skidpad":           0.0,
    "acceleration":      0.0,
}


def _generate_and_launch(context, *args, **kwargs):  # noqa: ANN001
    """OpaqueFunction: generate the track then return the Gazebo launch action."""

    track_type   = LaunchConfiguration("track_type").perform(context).strip()
    render_mode  = LaunchConfiguration("render_mode").perform(context).strip()
    target_length_m = float(LaunchConfiguration("target_length_m").perform(context))
    gz_headless  = LaunchConfiguration("gz_headless").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    launch_rviz  = LaunchConfiguration("launch_rviz").perform(context)
    map_file     = LaunchConfiguration("map_file").perform(context).strip()

    fs_vehicle_launch = (
        Path(get_package_share_directory("fs_vehicle")) / "launch" / "spawn_vehicle.launch.py"
    )

    # ── Custom map bypass ─────────────────────────────────────────────────────
    # When map_file is supplied, skip track generation entirely and launch
    # Gazebo directly with the provided world file.
    if map_file:
        world_name = Path(map_file).stem
        return [
            LogInfo(msg=f"[sim] Custom map: {Path(map_file).name}  render={render_mode}"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(fs_vehicle_launch)),
                launch_arguments={
                    "world":       map_file,
                    "world_name":  world_name,
                    "gz_headless": gz_headless,
                    "use_sim_time": use_sim_time,
                    "launch_rviz": launch_rviz,
                }.items(),
            ),
        ]

    # ── Generated track ───────────────────────────────────────────────────────
    from fs_track_generator.pipeline import generate_versioned_track

    # For fixed-geometry events, target_length is irrelevant; use the default.
    if track_type in ("skidpad", "acceleration"):
        target_length_m = _DEFAULT_LENGTHS[track_type]
    elif target_length_m <= 0.0:
        target_length_m = _DEFAULT_LENGTHS.get(track_type, 450.0)

    result = generate_versioned_track(
        track_type=track_type,
        seed=0,
        target_length_m=target_length_m,
        weather="dry",
        cone_noise="low",
        min_turn_radius_m=4.0,
        output_dir=None,
        render_mode=render_mode,
        delete_on_stop=True,
    )
    version    = result["version"]
    paths      = result["paths"]
    layout     = result["layout"]
    world_name = result["world_name"]
    model_name = result["model_name"]

    start = layout.start_pose

    def _cleanup_generated_files(context, *args, **kwargs):  # noqa: ANN001
        from fs_track_generator.artifacts import delete_generated_artifacts

        delete_generated_artifacts(paths)
        return []

    return [
        LogInfo(msg=(
            f"[sim] Generated {track_type} v{version}  "
            f"cones={len(layout.cones)}  "
            f"length={layout.track_length_m:.0f} m  render={render_mode}"
        )),
        LogInfo(msg=(
            f"[sim] Start pose  "
            f"x={start.x:.2f}  y={start.y:.2f}  "
            f"z={start.z:.2f}  yaw={start.yaw:.3f} rad"
        )),
        LogInfo(msg=f"[sim] World={world_name}  Model={model_name}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(fs_vehicle_launch)),
            launch_arguments={
                "world":                  str(paths["world"]),
                "spawn_x":                str(start.x),
                "spawn_y":                str(start.y),
                "spawn_z":                str(start.z),
                "spawn_yaw":              str(start.yaw),
                "model_name":             model_name,
                "world_name":             world_name,
                "bridge_config":          str(paths["bridge"]),
                "sensors_bridge_config":  str(paths["bridge_sensors"]),
                "gz_headless":            gz_headless,
                "use_sim_time":           use_sim_time,
                "launch_rviz":            launch_rviz,
            }.items(),
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    OpaqueFunction(function=_cleanup_generated_files),
                ],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "track_type",
            default_value="trackdrive",
            description=(
                "Track type: trackdrive | test_trackdrive | skidpad | acceleration"
            ),
        ),
        DeclareLaunchArgument(
            "render_mode",
            default_value="performance",
            description="Gazebo render mode: performance (colored cylinders) | high (DAE meshes)",
        ),
        DeclareLaunchArgument(
            "target_length_m",
            default_value="0.0",
            description=(
                "Target centreline length in metres "
                "(trackdrive/test_trackdrive only; 0 = use type default)"
            ),
        ),
        DeclareLaunchArgument(
            "map_file",
            default_value="",
            description=(
                "Absolute path to a custom .world or .sdf file. "
                "When set, track generation is skipped entirely."
            ),
        ),
        DeclareLaunchArgument(
            "gz_headless",
            default_value="false",
            description="Run Gazebo without a GUI (useful for CI / batch testing)",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo simulation clock for all ROS nodes",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Launch RViz automatically with the simulator",
        ),
        OpaqueFunction(function=_generate_and_launch),
    ])
