"""Console entry point for generating versioned track files."""

from __future__ import annotations

import argparse

from .pipeline import generate_versioned_track


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate a Formula Student Gazebo track")
    parser.add_argument("--seed", type=int, default=42, help="Deterministic seed for track generation")
    parser.add_argument(
        "--type",
        dest="track_type",
        choices=("trackdrive", "test_trackdrive", "skidpad", "acceleration"),
        default="trackdrive",
        help=(
            "Track type to generate: "
            "trackdrive (closed loop, default ~450 m), "
            "test_trackdrive (short closed loop, default ~150 m, higher RTF), "
            "skidpad (FSG figure-eight, fixed geometry), "
            "acceleration (FSG 75 m straight, fixed geometry)"
        ),
    )
    parser.add_argument(
        "--track-length",
        type=float,
        default=450.0,
        help=(
            "Target centerline length in meters "
            "(trackdrive / test_trackdrive only; ignored for skidpad and acceleration)"
        ),
    )
    parser.add_argument(
        "--min-turn-radius",
        type=float,
        default=5.0,
        help="Minimum acceptable turn radius in meters",
    )
    parser.add_argument(
        "--cone-noise",
        choices=("low", "medium", "high"),
        default="low",
        help="Metadata only for now; reserved for the future TUI",
    )
    parser.add_argument(
        "--weather",
        choices=("dry", "wet"),
        default="dry",
        help="Ground friction preset for the generated world",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Override the directory used to store generated maps",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    result = generate_versioned_track(
        track_type=args.track_type,
        seed=args.seed,
        target_length_m=args.track_length,
        weather=args.weather,
        cone_noise=args.cone_noise,
        min_turn_radius_m=args.min_turn_radius,
        output_dir=args.output_dir,
    )
    version = result["version"]
    paths = result["paths"]
    layout = result["layout"]
    validation = result["validation"]

    print(f"Generated version {version}")
    print(f"World: {paths['world']}")
    print(f"Cones: {paths['cones']}")
    print(f"Meta:  {paths['meta']}")
    print(f"Start pose: {layout.start_pose.to_dict()}")
    print(f"Validation: {validation['checks']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
