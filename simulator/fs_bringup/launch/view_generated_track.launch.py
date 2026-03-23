"""Launch a generated track world and spawn the car at the saved start pose."""

from __future__ import annotations

import json
import re
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

WORLD_NAME_RE = re.compile(r"map_layout_generated_(\d+)\.world$")


def _default_generated_dir() -> Path:
    launch_file = Path(__file__).resolve()

    for parent in launch_file.parents:
        source_dir = parent / "src" / "simulator" / "fs_track" / "worlds" / "generated"
        if source_dir.exists():
            return source_dir

        installed_dir = parent / "fs_track" / "worlds" / "generated"
        if installed_dir.exists():
            return installed_dir

    return Path(get_package_share_directory("simulator")) / "fs_track" / "worlds" / "generated"


def _resolve_generated_dir(output_dir: str | None = None) -> Path:
    chosen = Path(output_dir).expanduser().resolve() if output_dir else _default_generated_dir()
    chosen.mkdir(parents=True, exist_ok=True)
    return chosen


def _map_file_set(output_dir: Path, version: int) -> dict[str, Path]:
    stem = f"map_layout_generated_{version}"
    return {
        "world": output_dir / f"{stem}.world",
        "meta": output_dir / f"{stem}_meta.yaml",
        "cones": output_dir / f"{stem}_cones.yaml",
        "bridge": output_dir / f"{stem}_bridge.yaml",
    }


def _latest_generated_world(output_dir: str | None = None) -> dict[str, Path]:
    directory = _resolve_generated_dir(output_dir)
    latest_version = 0

    for world_file in directory.glob("map_layout_generated_*.world"):
        match = WORLD_NAME_RE.match(world_file.name)
        if match:
            latest_version = max(latest_version, int(match.group(1)))

    if latest_version == 0:
        raise FileNotFoundError(f"No generated maps found in {directory}")

    return _map_file_set(directory, latest_version)


def _read_structured_file(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _launch_generated_track(context, *args, **kwargs):
    version_text = LaunchConfiguration("version").perform(context).strip()
    generated_dir_text = LaunchConfiguration("generated_dir").perform(context).strip()

    output_dir = _resolve_generated_dir(generated_dir_text or None)

    if version_text:
        version = int(version_text)
        selected = _map_file_set(output_dir, version)
    else:
        selected = _latest_generated_world(str(output_dir))

    if not selected["world"].exists():
        raise FileNotFoundError(f"Generated world does not exist: {selected['world']}")
    if not selected["meta"].exists():
        raise FileNotFoundError(f"Generated metadata does not exist: {selected['meta']}")

    metadata = _read_structured_file(selected["meta"])
    start_pose = metadata["start_pose"]
    world_name = metadata.get("world_name", "fs_world")
    model_name = metadata.get("model_name", "fs_car")
    bridge_file = Path(metadata.get("bridge_file", selected["bridge"]))
    fs_vehicle_launch = Path(get_package_share_directory("fs_vehicle")) / "launch" / "spawn_vehicle.launch.py"

    if not bridge_file.exists():
        from fs_track_generator.ros_bridge import write_bridge_config

        write_bridge_config(
            bridge_file,
            world_name=world_name,
            model_name=model_name,
        )

    return [
        LogInfo(msg=f"Launching generated map: {selected['world']}"),
        LogInfo(msg=f"Using world={world_name} model={model_name}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(fs_vehicle_launch)),
            launch_arguments={
                "world": str(selected["world"]),
                "spawn_x": str(start_pose["x"]),
                "spawn_y": str(start_pose["y"]),
                "spawn_z": str(start_pose["z"]),
                "spawn_yaw": str(start_pose["yaw"]),
                "model_name": model_name,
                "world_name": world_name,
                "bridge_config": str(bridge_file),
                "gz_headless": LaunchConfiguration("gz_headless"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "version",
            default_value="",
            description="Generated map version to launch. Leave empty to launch the latest map.",
        ),
        DeclareLaunchArgument(
            "generated_dir",
            default_value="",
            description="Optional override for the generated maps directory.",
        ),
        DeclareLaunchArgument(
            "gz_headless",
            default_value="false",
            description="Run Gazebo without the GUI.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo simulation time.",
        ),
        OpaqueFunction(function=_launch_generated_track),
    ])
