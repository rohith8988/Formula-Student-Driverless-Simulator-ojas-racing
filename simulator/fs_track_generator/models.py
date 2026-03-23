"""Small dataclasses used by the track generator."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class Cone:
    """A single cone in world coordinates."""

    x: float
    y: float
    color: str
    size: str = "small"

    def to_dict(self) -> dict[str, float | str]:
        return {
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "color": self.color,
            "size": self.size,
        }


@dataclass(slots=True)
class StartPose:
    """Vehicle spawn pose for the generated track."""

    x: float
    y: float
    z: float
    yaw: float

    def to_dict(self) -> dict[str, float]:
        return {
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "z": round(self.z, 4),
            "yaw": round(self.yaw, 4),
        }


@dataclass(slots=True)
class TrackLayout:
    """In-memory representation of a generated track."""

    track_type: str
    seed: int
    target_length_m: float
    centerline: list[tuple[float, float]]
    left_boundary: list[tuple[float, float]]
    right_boundary: list[tuple[float, float]]
    cones: list[Cone]
    start_pose: StartPose
    track_length_m: float
    included_elements: list[str]
    weather: str
    cone_noise: str
