"""High-level helpers for generating and storing track artifacts."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .ros_bridge import write_bridge_configs
from .layout import generate_track_layout
from .world import build_world_sdf
from .artifacts import map_file_set, next_map_version, resolve_generated_dir, write_structured_file
from .validation import validate_track_layout

DEFAULT_MODEL_NAME = "fs_car"


def write_track_artifacts(
    *,
    version: int,
    paths: dict[str, Path],
    layout,
    validation: dict[str, Any],
    world_name: str,
    model_name: str,
    render_mode: str,
    delete_on_stop: bool,
) -> None:
    """Write the world, cone list, and metadata files for a generated track."""

    paths["world"].write_text(
        build_world_sdf(
            layout,
            world_name=world_name,
            render_mode=render_mode,
        ),
        encoding="utf-8",
    )
    write_structured_file(paths["cones"], {
        "track_type": layout.track_type,
        "seed": layout.seed,
        "cones": [cone.to_dict() for cone in layout.cones],
    })
    write_bridge_configs(
        paths["bridge"],
        paths["bridge_sensors"],
        world_name=world_name,
        model_name=model_name,
    )
    write_structured_file(paths["meta"], {
        "version": version,
        "track_type": layout.track_type,
        "seed": layout.seed,
        "weather": layout.weather,
        "cone_noise": layout.cone_noise,
        "world_name": world_name,
        "model_name": model_name,
        "render_mode": render_mode,
        "delete_on_stop": delete_on_stop,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "target_length_m": round(layout.target_length_m, 2),
        "track_length_m": round(layout.track_length_m, 2),
        "start_pose": layout.start_pose.to_dict(),
        "included_elements": layout.included_elements,
        "validation": validation,
        "world_file": str(paths["world"]),
        "cone_file": str(paths["cones"]),
        "bridge_file": str(paths["bridge"]),
        "bridge_sensors_file": str(paths["bridge_sensors"]),
        "meta_file": str(paths["meta"]),
    })


def generate_versioned_track(
    *,
    track_type: str,
    seed: int,
    target_length_m: float,
    weather: str,
    cone_noise: str,
    min_turn_radius_m: float,
    output_dir: str | None = None,
    model_name: str = DEFAULT_MODEL_NAME,
    render_mode: str = "high",
    delete_on_stop: bool = False,
) -> dict[str, Any]:
    """Generate a track, validate it, write versioned files, and return the result."""

    resolved_output_dir = resolve_generated_dir(output_dir)
    version = next_map_version(resolved_output_dir)
    paths = map_file_set(resolved_output_dir, version)
    world_name = f"fs_world_{version}"

    layout = generate_track_layout(
        track_type=track_type,
        seed=seed,
        target_length_m=target_length_m,
        weather=weather,
        cone_noise=cone_noise,
        min_turn_radius_m=min_turn_radius_m,
    )
    validation = validate_track_layout(layout, min_turn_radius_m=min_turn_radius_m)

    write_track_artifacts(
        version=version,
        paths=paths,
        layout=layout,
        validation=validation,
        world_name=world_name,
        model_name=model_name,
        render_mode=render_mode,
        delete_on_stop=delete_on_stop,
    )

    return {
        "version": version,
        "paths": paths,
        "layout": layout,
        "validation": validation,
        "world_name": world_name,
        "model_name": model_name,
        "render_mode": render_mode,
        "delete_on_stop": delete_on_stop,
    }
