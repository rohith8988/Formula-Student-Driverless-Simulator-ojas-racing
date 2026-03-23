"""Plain Python track generation logic."""

from __future__ import annotations

import math
from typing import Iterable

from .models import Cone, StartPose, TrackLayout
from .validation import validate_track_layout


# ── FSG skidpad constants (FSG Driverless Rules 2024, D4.1) ──────────
# D4.1.2: centres 18.25 m apart; inner circle Ø15.25 m; outer circle Ø21.25 m
_SKIDPAD_HALF_SPAN = 9.125    # m — origin to each circle centre (18.25 / 2)
_SKIDPAD_INNER_R   = 7.625    # m — inner boundary radius  (Ø15.25 m / 2)
_SKIDPAD_OUTER_R   = 10.625   # m — outer boundary radius  (Ø21.25 m / 2)
# D4.1.3: 16 inner cones per circle, 13 outer cones per circle
_SKIDPAD_N_INNER = 16
_SKIDPAD_N_OUTER = 13
# Crossing corridor: the approach/exit path runs along y ≈ 0.
# No boundary cones inside this rectangular zone — gate cones define it.
# _CORRIDOR_HALF_Y = 2.2 catches inner boundary cones at y ≈ ±2.07 m (one
# step away from the tangent point) in addition to the tangent cones at ±1.5 m.
_SKIDPAD_CORRIDOR_HALF_Y  = 2.2   # m — excludes inner boundary cones at ≈ ±2.07 m
_SKIDPAD_CORRIDOR_X_LIMIT = 6.5   # m — catches outer boundary cones at x ≈ ±6.04 m

# ── FSG acceleration constants ────────────────────────────────────────
_ACCEL_LENGTH_M = 75.0
_ACCEL_HALF_WIDTH_M = 1.5   # 3m total lane width


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _total_length(points: list[tuple[float, float]]) -> float:
    return sum(_distance(first, second) for first, second in zip(points, points[1:]))


def _translate(points: Iterable[tuple[float, float]], dx: float, dy: float) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in points]


def _scale(points: Iterable[tuple[float, float]], scale: float) -> list[tuple[float, float]]:
    return [(x * scale, y * scale) for x, y in points]


def _catmull_rom_closed(
    points: list[tuple[float, float]],
    samples_per_segment: int = 40,
    alpha: float = 0.5,
) -> list[tuple[float, float]]:
    """Closed centripetal Catmull-Rom spline (Barry & Goldman algorithm).

    alpha=0.5 (centripetal) eliminates cusps and curvature spikes that occur
    with uniform parameterisation when adjacent control points are spaced
    unevenly — for example, a short return segment next to a long main straight.
    """

    def _knot(pa: tuple[float, float], pb: tuple[float, float], t_prev: float) -> float:
        d = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
        return t_prev + (d ** alpha if d > 1e-10 else 1e-10)

    def _lerp(
        a: tuple[float, float],
        b: tuple[float, float],
        ta: float,
        tb: float,
        t: float,
    ) -> tuple[float, float]:
        span = tb - ta
        if abs(span) < 1e-12:
            return a
        w = (t - ta) / span
        return (a[0] + w * (b[0] - a[0]), a[1] + w * (b[1] - a[1]))

    sampled: list[tuple[float, float]] = []
    count = len(points)

    for index in range(count):
        p0 = points[(index - 1) % count]
        p1 = points[index]
        p2 = points[(index + 1) % count]
        p3 = points[(index + 2) % count]

        t0 = 0.0
        t1 = _knot(p0, p1, t0)
        t2 = _knot(p1, p2, t1)
        t3 = _knot(p2, p3, t2)

        if abs(t2 - t1) < 1e-12:
            continue  # degenerate (duplicate control points) — skip segment

        for step in range(samples_per_segment):
            t = t1 + (t2 - t1) * (step / samples_per_segment)
            a1 = _lerp(p0, p1, t0, t1, t)
            a2 = _lerp(p1, p2, t1, t2, t)
            a3 = _lerp(p2, p3, t2, t3, t)
            b1 = _lerp(a1, a2, t0, t2, t)
            b2 = _lerp(a2, a3, t1, t3, t)
            sampled.append(_lerp(b1, b2, t1, t2, t))

    sampled.append(sampled[0])
    return sampled


def _resample_polyline(points: list[tuple[float, float]], spacing: float) -> list[tuple[float, float]]:
    if len(points) < 2:
        return points

    resampled = [points[0]]
    next_target = spacing
    traversed = 0.0

    for first, second in zip(points, points[1:]):
        segment_length = _distance(first, second)
        if segment_length == 0.0:
            continue

        while traversed + segment_length >= next_target:
            ratio = (next_target - traversed) / segment_length
            resampled.append((
                first[0] + (second[0] - first[0]) * ratio,
                first[1] + (second[1] - first[1]) * ratio,
            ))
            next_target += spacing

        traversed += segment_length

    if _distance(resampled[-1], points[-1]) > 0.2:
        resampled.append(points[-1])

    if _distance(resampled[-1], resampled[0]) > 1e-6:
        resampled.append(resampled[0])

    return resampled


def _unit_tangent(points: list[tuple[float, float]], index: int) -> tuple[float, float]:
    previous_point = points[index - 1]
    next_point = points[(index + 1) % len(points)]
    dx = next_point[0] - previous_point[0]
    dy = next_point[1] - previous_point[1]
    length = math.hypot(dx, dy)
    if length == 0:
        return (1.0, 0.0)
    return (dx / length, dy / length)


def _point_along(points: list[tuple[float, float]], distance_m: float) -> tuple[float, float]:
    travelled = 0.0
    for first, second in zip(points, points[1:]):
        segment_length = _distance(first, second)
        if travelled + segment_length >= distance_m:
            ratio = (distance_m - travelled) / segment_length if segment_length else 0.0
            return (
                first[0] + (second[0] - first[0]) * ratio,
                first[1] + (second[1] - first[1]) * ratio,
            )
        travelled += segment_length
    return points[-1]


# ── Hardcoded FSG circuit templates ─────────────────────────────────────────
#
# Points are normalised — the generator scales them to target_length_m.
# Layout reads:  main straight (right) → sweeping right hairpin → S-curve
#                section through the middle → left sweep → return to start.
# Inspired by FSG Trackdrive circuit geometry (D-shape with S-curves).
# No randomisation: every launch of the same track_type looks identical.

_TRACKDRIVE_TEMPLATE: list[tuple[float, float]] = [
    ( 0.00,  0.00),  # start/finish — car heads +X
    ( 0.37,  0.00),  # main straight ⅓
    ( 0.74,  0.00),  # main straight ⅔
    ( 1.10,  0.00),  # end of main straight
    ( 1.40, -0.15),  # entry sweep into first hairpin
    ( 1.60, -0.45),  # right hairpin approach
    ( 1.50, -0.85),  # right hairpin apex
    ( 1.10, -1.00),  # hairpin exit, heading left
    ( 0.70, -0.85),  # S-curve — left peak
    ( 0.40, -0.60),  # S-curve centre
    ( 0.15, -0.80),  # S-curve — right peak
    (-0.10, -0.55),  # left sweep approach
    (-0.20, -0.25),  # left sweep apex
    (-0.20,  0.00),  # final straight approach — heading east
]

_TEST_TRACKDRIVE_TEMPLATE: list[tuple[float, float]] = [
    ( 0.00,  0.00),  # start/finish — heads +X
    ( 0.25,  0.00),  # main straight ⅓
    ( 0.50,  0.00),  # main straight ⅔
    ( 0.75, -0.05),  # end of straight / entry
    ( 0.90, -0.20),  # right curve entry
    ( 0.90, -0.50),  # right hairpin apex
    ( 0.75, -0.70),  # right hairpin exit
    ( 0.45, -0.80),  # back straight mid
    ( 0.15, -0.80),  # back straight end
    (-0.10, -0.65),  # left curve entry
    (-0.20, -0.35),  # left curve apex
    (-0.15, -0.05),  # left curve exit / return
]


def _load_circuit_centerline(
    template: list[tuple[float, float]],
    target_length_m: float,
) -> list[tuple[float, float]]:
    """Fit a hardcoded circuit template to the requested track length."""
    curve = _catmull_rom_closed(template)
    curve = _translate(curve, -curve[0][0], -curve[0][1])
    scale = target_length_m / max(_total_length(curve), 1e-6)
    return _resample_polyline(_scale(curve, scale), spacing=1.25)


def _generate_skidpad_layout() -> tuple[list[Cone], list[tuple[float, float]], "StartPose"]:
    """Generate FSG skidpad per D4.1 (FSG Driverless Rules 2024).

    Geometry (D4.1.2):
      - Right circle centre: (0, +9.125 m) — car drives clockwise
      - Left  circle centre: (0, -9.125 m) — car drives counter-clockwise
      - Inner boundary radius: 7.625 m  (diameter 15.25 m)
      - Outer boundary radius: 10.625 m (diameter 21.25 m)
      - Driving lane: 3 m wide, centreline radius = 9.125 m
      - Circles are tangent at origin — the timekeeping / crossing zone
      - Car approaches from +X, circles are on the ±Y axis

    Crossing corridor (y ∈ [−1.5, +1.5], x ∈ [−6.5, +6.5]):
      The outer boundary circles reach x ≈ ±6.04 m at y ≈ ±0.39 m near the
      crossing.  Any boundary cone inside the rectangular zone |y| < 2.2 m AND
      |x| < 6.5 m is excluded so the crossing remains completely free.

    Approach / exit: open lane — NO corridor cones on entry or exit path.

    Cone counts (D4.1.3): 16 inner orange, 13 outer (blue=right / yellow=left)
    """
    right_ctr = (0.0, +_SKIDPAD_HALF_SPAN)
    left_ctr  = (0.0, -_SKIDPAD_HALF_SPAN)
    entry_len = 12.0   # distance from origin to start/end of approach lane

    def _in_crossing_zone(x: float, y: float) -> bool:
        """True if the point falls inside the approach/exit corridor."""
        return abs(y) < _SKIDPAD_CORRIDOR_HALF_Y and abs(x) < _SKIDPAD_CORRIDOR_X_LIMIT

    cones: list[Cone] = []

    # ── Inner boundary cones (16 per circle, orange) ──────────────────
    for i in range(_SKIDPAD_N_INNER):
        a = 2 * math.pi * i / _SKIDPAD_N_INNER
        ri_x = right_ctr[0] + _SKIDPAD_INNER_R * math.cos(a)
        ri_y = right_ctr[1] + _SKIDPAD_INNER_R * math.sin(a)
        if not _in_crossing_zone(ri_x, ri_y):
            cones.append(Cone(ri_x, ri_y, "orange"))

        li_x = left_ctr[0] + _SKIDPAD_INNER_R * math.cos(a)
        li_y = left_ctr[1] + _SKIDPAD_INNER_R * math.sin(a)
        if not _in_crossing_zone(li_x, li_y):
            cones.append(Cone(li_x, li_y, "orange"))

    # ── Outer boundary cones (13 per circle, blue=right / yellow=left) ─
    for i in range(_SKIDPAD_N_OUTER):
        a = 2 * math.pi * i / _SKIDPAD_N_OUTER
        ro_x = right_ctr[0] + _SKIDPAD_OUTER_R * math.cos(a)
        ro_y = right_ctr[1] + _SKIDPAD_OUTER_R * math.sin(a)
        if not _in_crossing_zone(ro_x, ro_y):
            cones.append(Cone(ro_x, ro_y, "blue"))

        lo_x = left_ctr[0] + _SKIDPAD_OUTER_R * math.cos(a)
        lo_y = left_ctr[1] + _SKIDPAD_OUTER_R * math.sin(a)
        if not _in_crossing_zone(lo_x, lo_y):
            cones.append(Cone(lo_x, lo_y, "yellow"))

    # ── Timekeeping gate (large orange at the line between circle centres) ─
    # D4.1.6: the line between centres (Y-axis, x=0) is the start/finish line.
    cones.append(Cone(0.0, +1.9, "orange", size="large"))
    cones.append(Cone(0.0, -1.9, "orange", size="large"))

    # ── Reference centreline: entry → right CW → left CCW → exit ──────
    centerline: list[tuple[float, float]] = []
    for i in range(10):
        centerline.append((entry_len * (1.0 - i / 9.0), 0.0))
    # Right circle CW — start at bottom (angle = −π/2 from centre = origin)
    for i in range(73):
        a = -math.pi / 2.0 - 2.0 * math.pi * i / 72.0
        centerline.append((right_ctr[0] + _SKIDPAD_HALF_SPAN * math.cos(a),
                            right_ctr[1] + _SKIDPAD_HALF_SPAN * math.sin(a)))
    # Left circle CCW — start at top (angle = +π/2 from centre = origin)
    for i in range(73):
        a = math.pi / 2.0 + 2.0 * math.pi * i / 72.0
        centerline.append((left_ctr[0] + _SKIDPAD_HALF_SPAN * math.cos(a),
                            left_ctr[1] + _SKIDPAD_HALF_SPAN * math.sin(a)))
    # Exit leg to −X
    for i in range(10):
        centerline.append((-entry_len * i / 9.0, 0.0))

    start_pose = StartPose(x=entry_len, y=0.0, z=0.5, yaw=math.pi)
    return cones, centerline, start_pose


def _generate_acceleration_layout() -> tuple[list[Cone], list[tuple[float, float]], "StartPose"]:
    """Generate FSG acceleration event: 75 m straight, 3 m wide.

    Car starts at x = -3 m (behind the start line) and accelerates along +X.
    Large orange gate cones mark start (x=0) and finish (x=75).
    Small blue / yellow cones line the sides every 5 m.
    """
    cones: list[Cone] = []

    # Side cones every 5 m
    x = 0.0
    while x <= _ACCEL_LENGTH_M + 0.01:
        cones.append(Cone(x,  _ACCEL_HALF_WIDTH_M, "blue"))
        cones.append(Cone(x, -_ACCEL_HALF_WIDTH_M, "yellow"))
        x += 5.0

    # Start gate (x = 0)
    cones.append(Cone(0.0,  _ACCEL_HALF_WIDTH_M + 0.3, "orange", size="large"))
    cones.append(Cone(0.0, -(_ACCEL_HALF_WIDTH_M + 0.3), "orange", size="large"))
    # Finish gate (x = 75)
    cones.append(Cone(_ACCEL_LENGTH_M,  _ACCEL_HALF_WIDTH_M + 0.3, "orange", size="large"))
    cones.append(Cone(_ACCEL_LENGTH_M, -(_ACCEL_HALF_WIDTH_M + 0.3), "orange", size="large"))

    centerline = [(float(i), 0.0) for i in range(int(_ACCEL_LENGTH_M) + 1)]
    start_pose = StartPose(x=-3.0, y=0.0, z=0.5, yaw=0.0)
    return cones, centerline, start_pose


def _build_boundaries(points: list[tuple[float, float]], half_width_m: float = 1.5) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    left_boundary: list[tuple[float, float]] = []
    right_boundary: list[tuple[float, float]] = []

    for index in range(len(points)):
        tangent_x, tangent_y = _unit_tangent(points, index)
        normal_x = -tangent_y
        normal_y = tangent_x
        x, y = points[index]
        left_boundary.append((x + normal_x * half_width_m, y + normal_y * half_width_m))
        right_boundary.append((x - normal_x * half_width_m, y - normal_y * half_width_m))

    return left_boundary, right_boundary


def _local_curvature(points: list[tuple[float, float]], index: int) -> float:
    if index == 0 or index >= len(points) - 1:
        return 0.0

    a = points[index - 1]
    b = points[index]
    c = points[index + 1]

    ab = _distance(a, b)
    bc = _distance(b, c)
    ac = _distance(a, c)
    area = abs(
        0.5 * (
            a[0] * (b[1] - c[1]) +
            b[0] * (c[1] - a[1]) +
            c[0] * (a[1] - b[1])
        )
    )
    if area < 1e-6 or ab * bc * ac == 0:
        return 0.0

    radius = (ab * bc * ac) / (4.0 * area)
    return 0.0 if radius == 0 else 1.0 / radius


def _cone_spacing(points: list[tuple[float, float]], index: int, track_type: str) -> float:
    if track_type not in ("trackdrive", "test_trackdrive"):
        return 2.5   # fixed-geometry events use uniform 2.5 m spacing

    curvature = min(_local_curvature(points, index), 0.25)
    blend = curvature / 0.25
    straight_spacing = 5.0   # FSG 2024 max: 5 m on straights
    corner_spacing = 2.5     # FSG 2024 max: 3 m on corners (2.5 m passes hairpin overlap validation)
    return straight_spacing + (corner_spacing - straight_spacing) * blend


def _place_cones(
    centerline: list[tuple[float, float]],
    left_boundary: list[tuple[float, float]],
    right_boundary: list[tuple[float, float]],
    track_type: str,
) -> list[Cone]:
    cones: list[Cone] = []
    cones.extend(_place_side_cones(centerline, left_boundary, "blue", track_type))
    cones.extend(_place_side_cones(centerline, right_boundary, "yellow", track_type))
    return cones


def _place_side_cones(
    centerline: list[tuple[float, float]],
    boundary: list[tuple[float, float]],
    color: str,
    track_type: str,
) -> list[Cone]:
    cones: list[Cone] = []
    carried = 0.0
    spacing = _cone_spacing(centerline, 1, track_type)

    for index in range(1, len(boundary)):
        start = boundary[index - 1]
        end = boundary[index]
        segment_length = _distance(start, end)

        while carried + segment_length >= spacing and segment_length > 0.0:
            ratio = (spacing - carried) / segment_length
            cone_x = start[0] + (end[0] - start[0]) * ratio
            cone_y = start[1] + (end[1] - start[1]) * ratio
            cones.append(Cone(cone_x, cone_y, color))

            start = (cone_x, cone_y)
            segment_length = _distance(start, end)
            carried = 0.0
            spacing = _cone_spacing(centerline, min(index, len(centerline) - 2), track_type)

        carried += segment_length

    return cones


def _start_finish_cones(
    centerline: list[tuple[float, float]],
    left_boundary: list[tuple[float, float]],
    right_boundary: list[tuple[float, float]],
) -> list[Cone]:
    start_center = centerline[0]
    gate_center = _point_along(centerline, 5.0)

    start_pair = [
        Cone(left_boundary[0][0], left_boundary[0][1], "orange", size="large"),
        Cone(right_boundary[0][0], right_boundary[0][1], "orange", size="large"),
    ]

    tangent_x, tangent_y = _unit_tangent(centerline, 2)
    normal_x = -tangent_y
    normal_y = tangent_x

    gate_pair = [
        Cone(gate_center[0] + normal_x * 1.5, gate_center[1] + normal_y * 1.5, "orange", size="large"),
        Cone(gate_center[0] - normal_x * 1.5, gate_center[1] - normal_y * 1.5, "orange", size="large"),
        Cone(start_center[0] + tangent_x * 2.5 + normal_x * 1.9, start_center[1] + tangent_y * 2.5 + normal_y * 1.9, "orange"),
        Cone(start_center[0] + tangent_x * 2.5 - normal_x * 1.9, start_center[1] + tangent_y * 2.5 - normal_y * 1.9, "orange"),
    ]

    return start_pair + gate_pair


def generate_track_layout(
    *,
    track_type: str,
    seed: int,
    target_length_m: float,
    weather: str = "dry",
    cone_noise: str = "low",
    min_turn_radius_m: float = 4.0,
) -> TrackLayout:
    """Build a deterministic track layout from simple seeded rules.

    Supported track_type values:
      trackdrive      – FSG Trackdrive closed loop (~450 m, seeded random layout)
      test_trackdrive – Short closed loop (~150 m) for high-RTF testing; same
                        random generator as trackdrive, fewer cones
      skidpad         – FSG figure-eight (fixed geometry, seed / length ignored)
      acceleration    – FSG 75 m straight sprint (fixed geometry)
    """
    left_boundary: list[tuple[float, float]] = []
    right_boundary: list[tuple[float, float]] = []

    if track_type == "skidpad":
        cones, centerline, start_pose = _generate_skidpad_layout()
        included_elements = ["figure_eight", "constant_radius_corner"]

    elif track_type == "acceleration":
        cones, centerline, start_pose = _generate_acceleration_layout()
        included_elements = ["straight"]

    elif track_type in ("trackdrive", "test_trackdrive"):
        template = (
            _TRACKDRIVE_TEMPLATE
            if track_type == "trackdrive"
            else _TEST_TRACKDRIVE_TEMPLATE
        )
        centerline = _load_circuit_centerline(template, target_length_m)
        included_elements = ["straight", "constant_radius_corner", "variable_radius_curve"]
        left_boundary, right_boundary = _build_boundaries(centerline)
        sf_cones = _start_finish_cones(centerline, left_boundary, right_boundary)
        sf_positions = [(c.x, c.y) for c in sf_cones]
        boundary_cones = _place_cones(centerline, left_boundary, right_boundary, track_type)
        cones = [
            c for c in boundary_cones
            if all(_distance((c.x, c.y), sfp) >= 0.2 for sfp in sf_positions)
        ] + sf_cones
        tangent_x, tangent_y = _unit_tangent(centerline, 2)
        start_pose = StartPose(
            x=centerline[0][0],
            y=centerline[0][1],
            z=0.5,
            yaw=math.atan2(tangent_y, tangent_x),
        )

    else:
        raise ValueError(
            f"Unknown track_type: {track_type!r}. "
            "Valid: trackdrive, test_trackdrive, skidpad, acceleration"
        )

    layout = TrackLayout(
        track_type=track_type,
        seed=seed,
        target_length_m=target_length_m,
        centerline=centerline,
        left_boundary=left_boundary,
        right_boundary=right_boundary,
        cones=cones,
        start_pose=start_pose,
        track_length_m=_total_length(centerline),
        included_elements=included_elements,
        weather=weather,
        cone_noise=cone_noise,
    )

    validation = validate_track_layout(layout, min_turn_radius_m=min_turn_radius_m)
    if not validation["valid"]:
        raise ValueError(f"Generated track failed validation: {validation}")

    return layout
