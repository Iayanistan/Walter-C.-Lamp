"""Shake trajectory for D4607 10.5 ("shake the contents vigorously for 30 +/- 1 s")."""

from __future__ import annotations

import numpy as np


def shake_trajectory(q0, joint_index: int, amplitude: float, frequency: float, duration: float,
                     ramp: float = 0.5, rate: float = 50.0):
    """Sinusoidal oscillation of one joint about q0 with a smooth ramp in and out.

    Returns (times, positions) with positions shaped (N, len(q0)). The envelope is a raised cosine
    over `ramp` seconds at each end so the controller never sees a velocity step.
    """
    q0 = np.asarray(q0, dtype=float)
    t = np.arange(1, int(round(duration * rate)) + 1) / rate
    env = np.ones_like(t)
    if ramp > 0:
        up = t < ramp
        down = t > duration - ramp
        env[up] = 0.5 - 0.5 * np.cos(np.pi * t[up] / ramp)
        env[down] = 0.5 - 0.5 * np.cos(np.pi * (duration - t[down]) / ramp)
    positions = np.tile(q0, (len(t), 1))
    positions[:, joint_index] += env * amplitude * np.sin(2 * np.pi * frequency * t)
    return t, positions


def peak_velocity(amplitude: float, frequency: float) -> float:
    return 2 * np.pi * frequency * amplitude
