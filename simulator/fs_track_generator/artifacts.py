"""Helpers for locating and versioning generated track files."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any

try:
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
except ImportError:  # pragma: no cover - lets the generator run without ROS sourced
    PackageNotFoundError = RuntimeError
    get_package_share_directory = None


WORLD_NAME_RE = re.compile(r"map_layout_generated_(\d+)\.world$")


def _workspace_source_dir() -> Path | None:
    """Try to resolve the source-tree simulator directory from the current file path."""

    here = Path(__file__).resolve()

    for parent in here.parents:
        direct = parent / "fs_track" / "worlds" / "generated"
        if direct.parent.exists():
            return parent

        workspace = parent / "src" / "simulator" / "fs_track" / "worlds" / "generated"
        if workspace.parent.exists():
            return parent / "src" / "simulator"

    return None


def default_generated_dir() -> Path:
    """Return the preferred directory for storing generated map files."""

    source_dir = _workspace_source_dir()
    if source_dir is not None:
        return source_dir / "fs_track" / "worlds" / "generated"

    if get_package_share_directory is None:
        raise RuntimeError("Unable to resolve generated maps directory without ROS package lookup")

    try:
        return Path(get_package_share_directory("simulator")) / "fs_track" / "worlds" / "generated"
    except PackageNotFoundError as exc:  # pragma: no cover - only hit in broken environments
        raise RuntimeError("Unable to resolve a generated maps directory") from exc


def default_track_dir() -> Path:
    """Return the root directory that contains fs_track assets."""

    source_dir = _workspace_source_dir()
    if source_dir is not None:
        return source_dir / "fs_track"

    if get_package_share_directory is None:
        raise RuntimeError("Unable to resolve fs_track assets directory without ROS package lookup")

    try:
        return Path(get_package_share_directory("simulator")) / "fs_track"
    except PackageNotFoundError as exc:  # pragma: no cover - only hit in broken environments
        raise RuntimeError("Unable to resolve fs_track assets directory") from exc


def resolve_track_mesh_dir() -> Path:
    """Return the directory that contains the installed cone mesh assets."""

    mesh_dir = default_track_dir() / "meshes"
    if not mesh_dir.exists():
        raise RuntimeError(f"Could not find track mesh directory at {mesh_dir}")
    return mesh_dir


def resolve_generated_dir(output_dir: str | None = None) -> Path:
    """Pick a writable directory for generated maps."""

    chosen = Path(output_dir).expanduser().resolve() if output_dir else default_generated_dir()
    chosen.mkdir(parents=True, exist_ok=True)
    return chosen


def next_map_version(output_dir: Path) -> int:
    """Return the next numeric version for a generated map file."""

    latest = 0
    for world_file in output_dir.glob("map_layout_generated_*.world"):
        match = WORLD_NAME_RE.match(world_file.name)
        if match:
            latest = max(latest, int(match.group(1)))
    return latest + 1


def map_file_set(output_dir: Path, version: int) -> dict[str, Path]:
    """Return the world / cones / metadata file paths for a given version."""

    stem = f"map_layout_generated_{version}"
    return {
        "world":          output_dir / f"{stem}.world",
        "cones":          output_dir / f"{stem}_cones.yaml",
        "meta":           output_dir / f"{stem}_meta.yaml",
        "bridge":         output_dir / f"{stem}_bridge.yaml",
        "bridge_sensors": output_dir / f"{stem}_bridge_sensors.yaml",
    }


def latest_generated_world(output_dir: str | None = None) -> dict[str, Path]:
    """Return the latest generated world and its companion files."""

    directory = resolve_generated_dir(output_dir)
    latest_version = 0

    for world_file in directory.glob("map_layout_generated_*.world"):
        match = WORLD_NAME_RE.match(world_file.name)
        if match:
            latest_version = max(latest_version, int(match.group(1)))

    if latest_version == 0:
        raise FileNotFoundError(f"No generated maps found in {directory}")

    return map_file_set(directory, latest_version)


def write_structured_file(path: Path, payload: dict[str, Any]) -> None:
    """Write JSON content to a .yaml file so Python can read it back without extra deps."""

    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def read_structured_file(path: Path) -> dict[str, Any]:
    """Read a JSON-backed .yaml file."""

    return json.loads(path.read_text(encoding="utf-8"))


def default_maps_dir() -> Path:
    """Return the directory where user-provided custom map SDF/world files live."""

    source_dir = _workspace_source_dir()
    if source_dir is not None:
        return source_dir / "fs_track" / "worlds" / "maps"

    if get_package_share_directory is None:
        raise RuntimeError("Unable to resolve custom maps directory without ROS package lookup")

    try:
        return Path(get_package_share_directory("simulator")) / "fs_track" / "worlds" / "maps"
    except PackageNotFoundError as exc:  # pragma: no cover
        raise RuntimeError("Unable to resolve custom maps directory") from exc


def list_map_files(maps_dir: Path | None = None) -> list[Path]:
    """Return sorted list of .world and .sdf files in the custom maps directory."""

    directory = maps_dir if maps_dir is not None else default_maps_dir()
    if not directory.exists():
        return []
    return sorted(
        p for p in directory.iterdir()
        if p.suffix in (".world", ".sdf") and p.is_file()
    )


def delete_generated_artifacts(paths: dict[str, Path]) -> None:
    """Delete generated files if they exist."""

    for path in paths.values():
        try:
            path.unlink()
        except FileNotFoundError:
            continue
