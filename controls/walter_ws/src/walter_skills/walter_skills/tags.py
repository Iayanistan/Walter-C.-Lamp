"""Shared bits for the nodes that read AprilTags through TF."""

from __future__ import annotations

import time

from rclpy.duration import Duration
from rclpy.time import Time

from .placement import median_pose, yaw_from_quat


def tag_frame(fmt: str, tag_id: int) -> str:
    return fmt.format(id=tag_id)


def sample_relative(node, buffer, ref_frame: str, frame: str, n: int = 5, max_age_s: float = 0.5,
                    gap_s: float = 0.05):
    """Median [x, y, yaw_deg] of `frame` in `ref_frame`, or None if the tag is not currently seen."""
    out = []
    for _ in range(n):
        try:
            tf = buffer.lookup_transform(ref_frame, frame, Time())
        except Exception:  # noqa: BLE001 - tf2 raises several types; all mean "not seen"
            time.sleep(gap_s)
            continue
        age = (node.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        if age <= max_age_s:
            t, q = tf.transform.translation, tf.transform.rotation
            out.append([t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w) * 57.29577951308232])
        time.sleep(gap_s)
    return median_pose(out) if len(out) >= (n + 1) // 2 else None


def wait_for(buffer, ref_frame, frame, timeout_s=2.0):
    return buffer.can_transform(ref_frame, frame, Time(), timeout=Duration(seconds=timeout_s))

