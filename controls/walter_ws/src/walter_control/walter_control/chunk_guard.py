"""Checks one action chunk from the policy before it reaches the joint controller.

Rules, in order:
  1. shape and finite values
  2. clamp every point into the joint limits shrunk by `margin`
  3. reject if the first point is more than `max_start_jump` from the measured position
  4. reject if any point puts the TCP outside the workspace box
  5. stretch segment durations so no joint exceeds its velocity limit (retime, never speed up)

No ROS imports, so the same check can run in Isaac Sim or offline on recorded chunks.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .kinematics import tcp_position


@dataclass
class GuardConfig:
    lower: np.ndarray
    upper: np.ndarray
    max_velocity: np.ndarray  # already scaled, rad/s
    margin: float = 0.05
    max_start_jump: float = 0.15
    workspace_min: np.ndarray = field(default_factory=lambda: np.array([-np.inf] * 3))
    workspace_max: np.ndarray = field(default_factory=lambda: np.array([np.inf] * 3))
    robot: str = "irb120"  # which FK chain the workspace check uses (kinematics.CHAINS)


@dataclass
class GuardResult:
    ok: bool
    reason: str = ""
    positions: np.ndarray | None = None  # (N, J) after clamping
    times: np.ndarray | None = None  # (N,) seconds from now, after retiming
    retimed: bool = False
    clamped: bool = False


def check_chunk(current: np.ndarray, positions: np.ndarray, dt: float, cfg: GuardConfig) -> GuardResult:
    current = np.asarray(current, dtype=float)
    positions = np.asarray(positions, dtype=float)
    if positions.ndim != 2 or positions.shape[0] == 0 or positions.shape[1] != current.shape[0]:
        return GuardResult(False, f"bad chunk shape {positions.shape}, expected (N, {current.shape[0]})")
    if not np.all(np.isfinite(positions)):
        return GuardResult(False, "chunk contains NaN or inf")

    lo = np.asarray(cfg.lower) + cfg.margin
    hi = np.asarray(cfg.upper) - cfg.margin
    clamped_positions = np.clip(positions, lo, hi)
    clamped = bool(np.any(clamped_positions != positions))
    positions = clamped_positions

    jump = np.max(np.abs(positions[0] - current))
    if jump > cfg.max_start_jump:
        return GuardResult(False, f"first point is {jump:.3f} rad from the measured position "
                                  f"(limit {cfg.max_start_jump})")

    for i, q in enumerate(positions):
        p = tcp_position(q, cfg.robot)
        if np.any(p < cfg.workspace_min) or np.any(p > cfg.workspace_max):
            return GuardResult(False, f"point {i} puts the TCP at {np.round(p, 3).tolist()}, outside the workspace box")

    previous = np.vstack([current, positions[:-1]])
    needed = np.max(np.abs(positions - previous) / np.asarray(cfg.max_velocity), axis=1)
    segment = np.maximum(dt, needed)
    retimed = bool(np.any(segment > dt * (1 + 1e-6)))
    reason = ""
    if retimed:
        reason = f"slowed by x{segment.sum() / (dt * len(segment)):.2f} for velocity limits"
    if clamped:
        reason = (reason + "; " if reason else "") + "clamped to joint limits"
    return GuardResult(True, reason, positions, np.cumsum(segment), retimed, clamped)
