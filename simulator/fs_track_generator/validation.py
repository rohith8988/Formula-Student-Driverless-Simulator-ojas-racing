"""Validation helpers for generated track layouts."""

from __future__ import annotations

import math

from .models import TrackLayout



def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _circumcircle_radius(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
) -> float:
    ab = _distance(a, b)
    bc = _distance(b, c)
    ca = _distance(c, a)
    area = abs(
        0.5 * (
            a[0] * (b[1] - c[1]) +
            b[0] * (c[1] - a[1]) +
            c[0] * (a[1] - b[1])
        )
    )

    if area < 1e-6:
        return float("inf")

    return (ab * bc * ca) / (4.0 * area)


def _segment_intersection(
    a1: tuple[float, float],
    a2: tuple[float, float],
    b1: tuple[float, float],
    b2: tuple[float, float],
) -> bool:
    def orientation(p: tuple[float, float], q: tuple[float, float], r: tuple[float, float]) -> float:
        return (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])

    def on_segment(p: tuple[float, float], q: tuple[float, float], r: tuple[float, float]) -> bool:
        return (
            min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
            min(p[1], r[1]) <= q[1] <= max(p[1], r[1])
        )

    o1 = orientation(a1, a2, b1)
    o2 = orientation(a1, a2, b2)
    o3 = orientation(b1, b2, a1)
    o4 = orientation(b1, b2, a2)

    if (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0):
        return True

    if abs(o1) < 1e-9 and on_segment(a1, b1, a2):
        return True
    if abs(o2) < 1e-9 and on_segment(a1, b2, a2):
        return True
    if abs(o3) < 1e-9 and on_segment(b1, a1, b2):
        return True
    if abs(o4) < 1e-9 and on_segment(b1, a2, b2):
        return True

    return False


def _has_self_intersections(points: list[tuple[float, float]]) -> bool:
    segment_count = len(points) - 1
    for i in range(segment_count):
        for j in range(i + 2, segment_count):
            if i == 0 and j == segment_count - 1:
                continue
            if _segment_intersection(points[i], points[i + 1], points[j], points[j + 1]):
                return True
    return False


def validate_track_layout(
    layout: TrackLayout,
    *,
    width_tolerance_m: float = 0.15,
    min_turn_radius_m: float = 4.0,
) -> dict[str, object]:
    """Run geometric checks over a generated layout.

    Checks are applied conditionally depending on track_type:
      - width / boundary checks: only when left/right boundaries are present
        (trackdrive, test_trackdrive); skipped for skidpad and acceleration
      - closed-loop check: skipped for acceleration (open straight)
      - self-intersection check: skipped for skidpad (figure-8 is intentional)
      - cone spacing check: only for trackdrive / test_trackdrive
    """

    # ── Width check (boundary-based tracks only) ──────────────────────
    if layout.left_boundary and layout.right_boundary:
        widths = [
            _distance(left, right)
            for left, right in zip(layout.left_boundary, layout.right_boundary)
        ]
        min_width = min(widths)
        max_width = max(widths)
        width_ok = (
            abs(min_width - 3.0) <= width_tolerance_m
            and abs(max_width - 3.0) <= width_tolerance_m
        )
    else:
        min_width = max_width = 3.0   # nominal; not computed
        width_ok = True

    # ── Minimum turn radius ───────────────────────────────────────────
    if len(layout.centerline) >= 3:
        radii = [
            _circumcircle_radius(
                layout.centerline[i - 1],
                layout.centerline[i],
                layout.centerline[i + 1],
            )
            for i in range(1, len(layout.centerline) - 1)
        ]
        finite_radii = [r for r in radii if math.isfinite(r)]
        min_radius = min(finite_radii) if finite_radii else float("inf")
    else:
        min_radius = float("inf")
    radius_ok = min_radius >= min_turn_radius_m

    # ── Closed-loop check ─────────────────────────────────────────────
    # acceleration: open straight — no loop
    # skidpad: figure-8 has an entry lane so it doesn't close back to start
    if layout.track_type in ("acceleration", "skidpad"):
        closed_loop_ok = True  # open events — skip
    else:
        closed_loop_ok = _distance(layout.centerline[0], layout.centerline[-1]) < 2.0

    # ── Self-intersection (figure-8 intentionally crosses) ───────────
    if layout.track_type == "skidpad":
        self_intersection_ok = True
    else:
        self_intersection_ok = not _has_self_intersections(layout.centerline)

    # ── Cone spacing (only relevant for boundary-placed cones) ────────
    def spacing_ok(cones: list) -> bool:
        for first, second in zip(cones, cones[1:]):
            spacing = _distance((first.x, first.y), (second.x, second.y))
            if spacing < 1.4 or spacing > 5.2:
                return False
        return True

    if layout.track_type in ("skidpad", "acceleration"):
        spacing_rule_ok = True  # fixed-geometry events have their own spacing
    else:
        blue_cones = [c for c in layout.cones if c.color == "blue" and c.size == "small"]
        yellow_cones = [c for c in layout.cones if c.color == "yellow" and c.size == "small"]
        spacing_rule_ok = spacing_ok(blue_cones) and spacing_ok(yellow_cones)

    # ── Cone overlap check ────────────────────────────────────────────
    # Skidpad: the crossing point has cones from both circles at nearly the
    # same position by design — skip the overlap check for that type.
    if layout.track_type == "skidpad":
        overlaps_ok = True
    else:
        cone_positions = [(cone.x, cone.y) for cone in layout.cones]
        overlaps_ok = True
        for index, first in enumerate(cone_positions):
            for second in cone_positions[index + 1:]:
                if _distance(first, second) < 0.2:
                    overlaps_ok = False
                    break
            if not overlaps_ok:
                break

    checks = {
        "width_ok": width_ok,
        "radius_ok": radius_ok,
        "closed_loop_ok": closed_loop_ok,
        "self_intersection_ok": self_intersection_ok,
        "spacing_rule_ok": spacing_rule_ok,
        "overlaps_ok": overlaps_ok,
    }

    return {
        "valid": all(checks.values()),
        "checks": checks,
        "metrics": {
            "track_length_m": round(layout.track_length_m, 2),
            "min_width_m": round(min_width, 3),
            "max_width_m": round(max_width, 3),
            "min_turn_radius_m": round(min_radius, 3) if math.isfinite(min_radius) else None,
            "cone_count": len(layout.cones),
        },
    }
