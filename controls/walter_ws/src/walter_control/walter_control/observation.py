"""Turns ROS data into the nested observation dict GR00T expects, and actions back into targets.

GR00T observation layout (Isaac-GR00T getting_started/policy.md):
    video:    {camera: uint8 (B, T, H, W, 3)}
    state:    {name: float32 (B, T, D)}
    language: {annotation key: [[instruction]]}
Actions come back as {name: float32 (B, T, D)} in physical units.
"""

from __future__ import annotations

import numpy as np

_CHANNELS = {"rgb8": (3, False), "bgr8": (3, True), "rgba8": (4, False), "bgra8": (4, True)}


def image_to_rgb(encoding: str, height: int, width: int, step: int, data: bytes) -> np.ndarray:
    """sensor_msgs/Image fields -> (H, W, 3) uint8 RGB. Supports rgb8, bgr8, rgba8, bgra8."""
    if encoding not in _CHANNELS:
        raise ValueError(f"unsupported image encoding {encoding!r}")
    channels, bgr = _CHANNELS[encoding]
    rows = np.frombuffer(data, dtype=np.uint8).reshape(height, step)
    img = rows[:, : width * channels].reshape(height, width, channels)[:, :, :3]
    return np.ascontiguousarray(img[:, :, ::-1] if bgr else img)


def resize_nearest(img: np.ndarray, width: int, height: int) -> np.ndarray:
    if img.shape[0] == height and img.shape[1] == width:
        return img
    ys = (np.arange(height) * img.shape[0] / height).astype(int)
    xs = (np.arange(width) * img.shape[1] / width).astype(int)
    return img[ys][:, xs]


def build_observation(images: dict[str, np.ndarray], states: dict[str, np.ndarray],
                      instruction: str, language_key: str) -> dict:
    return {
        "video": {k: v[None, None].astype(np.uint8) for k, v in images.items()},
        "state": {k: np.asarray(v, dtype=np.float32)[None, None] for k, v in states.items()},
        "language": {language_key: [[instruction]]},
    }


def split_action(action: dict, arm_key: str, gripper_key: str | None, current_arm: np.ndarray,
                 deltas: bool) -> tuple[np.ndarray, np.ndarray | None]:
    """Returns (arm targets (T, 6), gripper targets (T,) or None) for batch element 0."""
    arm = np.asarray(action[arm_key], dtype=float)[0]
    if deltas:
        arm = arm + np.asarray(current_arm, dtype=float)
    grip = None
    if gripper_key and gripper_key in action:
        grip = np.asarray(action[gripper_key], dtype=float)[0].reshape(arm.shape[0], -1)[:, 0]
    return arm, grip
