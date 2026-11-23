"""Plane geometry for AprilTag placement checks (no ROS needed, so it is unit-tested).

Every tag lies flat on the bench, so a tag's pose relative to the reference tag is just
(x, y, yaw). Measuring against the reference tag instead of the camera means a bumped camera
does not break the checks.
"""

from __future__ import annotations

import math


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Rotation about z in radians (the only rotation a flat tag has relative to another)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def placement_error(measured, expected) -> tuple[float, float]:
    """measured/expected are [x_m, y_m, yaw_deg]. Returns (position error mm, |yaw error| deg)."""
    dx, dy = measured[0] - expected[0], measured[1] - expected[1]
    return math.hypot(dx, dy) * 1000.0, abs(wrap_deg(measured[2] - expected[2]))


def judge(measured, expected, tol_mm: float, tol_deg: float) -> tuple[bool, float, float, str]:
    mm, deg = placement_error(measured, expected)
    ok = mm <= tol_mm and deg <= tol_deg
    msg = (f"in place ({mm:.1f} mm, {deg:.1f} deg)" if ok else
           f"out of place by {mm:.1f} mm / {deg:.1f} deg (limit {tol_mm:.1f} mm / {tol_deg:.1f} deg)")
    return ok, mm, deg, msg


def median_pose(samples):
    """Median of [x, y, yaw_deg] samples; yaw is unwrapped around the first sample first."""
    if not samples:
        raise ValueError("no samples")
    ref = samples[0][2]
    xs = sorted(s[0] for s in samples)
    ys = sorted(s[1] for s in samples)
    yaws = sorted(ref + wrap_deg(s[2] - ref) for s in samples)
    mid = len(samples) // 2
    return [xs[mid], ys[mid], wrap_deg(yaws[mid])]
