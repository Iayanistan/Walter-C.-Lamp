"""Arm backends for the FR3 helpers.

Every helper in helpers.py talks to one of these, never to the SDK directly:

    DryRunArm     no robot. Tracks joint angles, prints what it would do, and (by default) runs on a
                  virtual clock so a 40-minute D4607 run finishes in a second. Use it first, always.
    FairinoArm    the real FR3 (or FAIRINO's SimMachine) through FAIRINO's Python SDK
                  (`from fairino import Robot`, controller V3.7.5 or newer).

Joint angles are radians everywhere in this package, like the ROS 2 side of the project. The SDK
wants degrees, so FairinoArm converts at the boundary.

FairinoArm is written against FAIRINO's published SDK reference (MoveJ, ServoMoveStart/ServoJ/
ServoMoveEnd, SetToolDO, SetDO, GetActualJointPosDegree, StopMotion) and checks the SDK's call
signatures at start-up because they changed between SDK releases. It has NOT been run against a
controller yet: try it on the SimMachine, then on the real arm at low speed, before trusting it.
"""

from __future__ import annotations

import inspect
import math
import time
from typing import Sequence

import numpy as np

# FR3 joint limits (rad), from fairino3_v6.urdf as used in walter_bringup/config/safety_fr3.yaml.
FR3_LOWER = np.array([-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543])
FR3_UPPER = np.array([3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543])
FR3_MAX_VEL = np.array([3.15, 3.15, 3.15, 3.2, 3.2, 3.2])   # rated joint speed, rad/s (about 180 deg/s)
JOINT_MARGIN = 0.05                                           # rad kept clear of each hard limit
SERVO_PERIOD = 0.008                                          # ServoJ cycle the FR3 expects (8 ms)


class ArmError(RuntimeError):
    pass


class Halted(ArmError):
    """Raised when the arm was stopped (spill detector, operator, or controller error)."""


def check_limits(q: Sequence[float], what: str = "target") -> np.ndarray:
    q = np.asarray(q, dtype=float)
    if q.shape != (6,):
        raise ArmError(f"{what}: need 6 joint values, got {q.shape}")
    lo, hi = FR3_LOWER + JOINT_MARGIN, FR3_UPPER - JOINT_MARGIN
    bad = [i + 1 for i in range(6) if not lo[i] <= q[i] <= hi[i]]
    if bad:
        raise ArmError(f"{what}: joint(s) {bad} outside the FR3 limits (with {JOINT_MARGIN} rad margin)")
    return q


def check_path(positions: np.ndarray, times: np.ndarray, q_start: np.ndarray, what: str):
    """Limits on every sample and joint speeds under 90% of rated, for servo paths."""
    for k in (0, len(positions) // 2, len(positions) - 1):
        check_limits(positions[k], what)
    if not (np.all(positions.min(axis=0) >= FR3_LOWER + JOINT_MARGIN)
            and np.all(positions.max(axis=0) <= FR3_UPPER - JOINT_MARGIN)):
        raise ArmError(f"{what}: path leaves the FR3 joint limits")
    pts = np.vstack([q_start, positions])
    dt = np.diff(np.concatenate([[0.0], times]))
    vel = np.abs(np.diff(pts, axis=0)) / dt[:, None]
    if np.any(vel > 0.9 * FR3_MAX_VEL):
        raise ArmError(f"{what}: path is faster than 90% of the FR3's rated joint speed")


class Arm:
    """Interface the helpers use. Subclasses implement the underscored methods."""

    halted_reason: str | None = None

    # --- time: the dry run swaps in a virtual clock ---
    def clock(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float):
        end = self.clock() + seconds
        while (left := end - self.clock()) > 0:
            self.raise_if_halted()
            time.sleep(min(left, 0.05))

    # --- motion ---
    def joints(self) -> np.ndarray:
        raise NotImplementedError

    def move_joints(self, q: Sequence[float], seconds: float, max_vel_pct: float = 30.0):
        """Joint move to q (rad), taking roughly `seconds`, never faster than max_vel_pct of rated."""
        self.raise_if_halted()
        self._move(check_limits(q), float(seconds), float(max_vel_pct))

    def follow(self, times: np.ndarray, positions: np.ndarray, what: str = "path"):
        """Stream a dense joint path (rad) at its own timing. Used for shake and swirl."""
        self.raise_if_halted()
        check_path(positions, times, self.joints(), what)
        self._follow(np.asarray(times, float), np.asarray(positions, float))

    def gripper(self, closed: bool):
        self.raise_if_halted()
        self._gripper(bool(closed))

    def pulse_do(self, do: int, seconds: float):
        self.raise_if_halted()
        self._pulse_do(int(do), float(seconds))

    def halt(self, reason: str):
        self.halted_reason = reason
        self._stop()

    def raise_if_halted(self):
        if self.halted_reason:
            raise Halted(self.halted_reason)

    def close(self):
        pass

    def _move(self, q, seconds, max_vel_pct):
        raise NotImplementedError

    def _follow(self, times, positions):
        raise NotImplementedError

    def _gripper(self, closed):
        raise NotImplementedError

    def _pulse_do(self, do, seconds):
        raise NotImplementedError

    def _stop(self):
        pass


class DryRunArm(Arm):
    """No robot. `realtime=False` (default) runs on a virtual clock: sleeps return at once."""

    def __init__(self, q0: Sequence[float] | None = None, realtime: bool = False, verbose: bool = True):
        self.q = check_limits(q0 if q0 is not None else [-0.3469, -1.7251, 1.3482, -1.1939, -1.5708, 0.0])
        self.realtime = realtime
        self.verbose = verbose
        self.closed = False
        self._t = 0.0
        self.trace: list[tuple] = []     # (time, kind, detail) for tests and the run log

    def clock(self):
        return time.monotonic() if self.realtime else self._t

    def sleep(self, seconds):
        if self.realtime:
            return super().sleep(seconds)
        self.raise_if_halted()
        self._t += max(0.0, seconds)

    def joints(self):
        return self.q.copy()

    def _say(self, kind, detail):
        self.trace.append((round(self.clock(), 3), kind, detail))
        if self.verbose:
            print(f"    [arm t={self.clock():8.1f}s] {kind}: {detail}")

    def _move(self, q, seconds, max_vel_pct):
        self._say("move", f"to {np.round(np.degrees(q), 1).tolist()} deg in {seconds:.1f} s")
        self.sleep(seconds)
        self.q = q

    def _follow(self, times, positions):
        self._say("servo", f"{len(times)} points over {times[-1]:.1f} s")
        self.sleep(float(times[-1]))
        self.q = positions[-1].copy()

    def _gripper(self, closed):
        self.closed = closed
        self._say("gripper", "close" if closed else "open")

    def _pulse_do(self, do, seconds):
        self._say("do", f"DO{do} high for {seconds} s")
        self.sleep(seconds)

    def _stop(self):
        self._say("STOP", self.halted_reason)


class FairinoArm(Arm):
    """The FR3 through FAIRINO's Python SDK. One SDK client: do not run fairino_hardware or
    fr_io_gripper (the ROS 2 path) against the same controller at the same time."""

    def __init__(self, ip: str = "192.168.58.2", tool: int = 0, user: int = 0,
                 gripper_do: int = 0, close_level: int = 1, enable: bool = True):
        try:
            from fairino import Robot
        except ImportError as e:  # pragma: no cover - needs the vendor SDK
            raise ArmError("FAIRINO's Python SDK is not installed (`from fairino import Robot` failed). "
                           "Install it as FAIRINO's SDK docs describe, or use --dry-run.") from e
        self.r = Robot.RPC(ip)
        self.tool, self.user = tool, user
        self.gripper_do, self.close_level = gripper_do, close_level
        if enable:   # teach.py only reads joint angles and leaves the mode alone
            self._ok(self.r.ResetAllError(), "ResetAllError")
            self._ok(self.r.Mode(0), "Mode(0) automatic")
            self._ok(self.r.RobotEnable(1), "RobotEnable(1)")
        # The SDK renamed and reordered arguments between releases; look before calling.
        self._movej_params = inspect.signature(self.r.MoveJ).parameters
        self._servoj_has_axis = "axisPos" in inspect.signature(self.r.ServoJ).parameters

    @staticmethod
    def _ok(ret, what):
        err = ret[0] if isinstance(ret, (tuple, list)) else ret
        if err != 0:
            raise ArmError(f"{what} failed with FAIRINO error code {err}")
        return ret

    def joints(self):
        ret = self._ok(self.r.GetActualJointPosDegree(1), "GetActualJointPosDegree")
        return np.radians(np.asarray(ret[1][:6], dtype=float))

    def _move(self, q, seconds, max_vel_pct):
        q_now = self.joints()
        # MoveJ takes a speed percentage, not a duration. Peak speed of a smooth move is about 1.5x
        # its average, so aim the percentage at the joint that has the farthest to go.
        need = np.max(np.abs(q - q_now) / FR3_MAX_VEL) / max(seconds, 0.1) * 1.5 * 100
        vel = float(np.clip(need, 1.0, max_vel_pct))
        kw = dict(joint_pos=[math.degrees(v) for v in q], tool=self.tool, user=self.user, vel=vel)
        if "desc_pos" in self._movej_params and self._movej_params["desc_pos"].default is inspect._empty:
            # Older SDKs want the Cartesian target too.
            fk = self._ok(self.r.GetForwardKin(kw["joint_pos"]), "GetForwardKin")
            kw["desc_pos"] = fk[1]
        self._ok(self.r.MoveJ(**kw), "MoveJ")   # blocking (blendT = -1 by default)

    def _follow(self, times, positions):
        self._ok(self.r.ServoMoveStart(), "ServoMoveStart")
        try:
            t0 = time.perf_counter()
            for t, q in zip(times, positions):
                if self.halted_reason:
                    raise Halted(self.halted_reason)
                deg = [math.degrees(v) for v in q]
                args = (deg, [0.0, 0.0, 0.0, 0.0]) if self._servoj_has_axis else (deg,)
                self._ok(self.r.ServoJ(*args, cmdT=SERVO_PERIOD), "ServoJ")
                lag = t0 + t - time.perf_counter()
                if lag > 0:
                    time.sleep(lag)
        finally:
            self.r.ServoMoveEnd()

    def _gripper(self, closed):
        level = self.close_level if closed else 1 - self.close_level
        self._ok(self.r.SetToolDO(self.gripper_do, level, 0, 0), f"SetToolDO({self.gripper_do}, {level})")

    def _pulse_do(self, do, seconds):
        self._ok(self.r.SetDO(do, 1, 0, 0), f"SetDO({do}, 1)")
        try:
            time.sleep(seconds)
        finally:
            self.r.SetDO(do, 0, 0, 0)

    def _stop(self):
        try:
            self.r.StopMotion()
        except Exception:  # noqa: BLE001 - stopping must never raise on top of the halt
            pass

    def close(self):
        try:
            self.r.CloseRPC()
        except Exception:  # noqa: BLE001
            pass
