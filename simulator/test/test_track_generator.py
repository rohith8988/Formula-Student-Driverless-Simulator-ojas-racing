from pathlib import Path

from fs_track_generator.layout import generate_track_layout
from fs_track_generator.world import build_world_sdf
from fs_track_generator.artifacts import (
    latest_generated_world,
    map_file_set,
    next_map_version,
    read_structured_file,
    resolve_generated_dir,
)
from fs_track_generator.validation import validate_track_layout
from fs_track_generator.pipeline import generate_versioned_track


def test_generate_trackdrive_layout_is_valid():
    layout = generate_track_layout(
        track_type="trackdrive",
        seed=42,
        target_length_m=450.0,
    )

    report = validate_track_layout(layout, min_turn_radius_m=4.0)
    world_sdf = build_world_sdf(layout)

    assert report["valid"] is True
    assert layout.track_type == "trackdrive"
    assert len(layout.cones) > 50
    assert '<world name="fs_world">' in world_sdf
    assert "cone_blue.dae" in world_sdf
    assert "cone_yellow.dae" in world_sdf
    assert "cone_big.dae" in world_sdf
    assert "<length>0.3250</length>" in world_sdf
    assert "<length>0.5050</length>" in world_sdf


def test_generate_test_trackdrive_layout_is_valid():
    layout = generate_track_layout(
        track_type="test_trackdrive",
        seed=0,
        target_length_m=300.0,
    )

    report = validate_track_layout(layout, min_turn_radius_m=4.0)

    assert report["valid"] is True
    assert layout.track_type == "test_trackdrive"
    assert 30 < len(layout.cones) < 180   # ~300 m track


def test_performance_render_mode_uses_simple_cones_and_disables_shadows():
    layout = generate_track_layout(
        track_type="trackdrive",
        seed=42,
        target_length_m=650.0,
    )

    world_sdf = build_world_sdf(layout, render_mode="performance")

    assert "cone_blue.dae" not in world_sdf
    assert "cone_yellow.dae" not in world_sdf
    assert "<shadows>false</shadows>" in world_sdf
    assert "<cast_shadows>false</cast_shadows>" in world_sdf
    assert "<cylinder>" in world_sdf


def test_versioned_map_names_increment(tmp_path: Path):
    output_dir = resolve_generated_dir(str(tmp_path))

    first_version = next_map_version(output_dir)
    first_paths = map_file_set(output_dir, first_version)
    first_paths["world"].write_text("<sdf/>", encoding="utf-8")

    second_version = next_map_version(output_dir)

    assert first_version == 1
    assert second_version == 2
    assert first_paths["world"].name == "map_layout_generated_1.world"


def test_latest_generated_world_uses_highest_numeric_version(tmp_path: Path):
    output_dir = resolve_generated_dir(str(tmp_path))

    map_file_set(output_dir, 2)["world"].write_text("<sdf/>", encoding="utf-8")
    map_file_set(output_dir, 10)["world"].write_text("<sdf/>", encoding="utf-8")

    latest = latest_generated_world(str(output_dir))

    assert latest["world"].name == "map_layout_generated_10.world"


def test_generate_versioned_track_writes_dynamic_bridge_metadata(tmp_path: Path):
    result = generate_versioned_track(
        track_type="trackdrive",
        seed=42,
        target_length_m=650.0,
        weather="dry",
        cone_noise="low",
        min_turn_radius_m=4.0,
        output_dir=str(tmp_path),
        render_mode="performance",
        delete_on_stop=True,
    )

    paths = result["paths"]
    metadata = read_structured_file(paths["meta"])
    bridge_text = paths["bridge"].read_text(encoding="utf-8")
    world_text = paths["world"].read_text(encoding="utf-8")

    assert metadata["world_name"] == "fs_world_1"
    assert metadata["model_name"] == "fs_car"
    assert metadata["render_mode"] == "performance"
    assert metadata["delete_on_stop"] is True
    assert metadata["bridge_file"] == str(paths["bridge"])
    assert '<world name="fs_world_1">' in world_text
    assert "<shadows>false</shadows>" in world_text
    assert '/world/fs_world_1/model/fs_car/joint_state' in bridge_text
