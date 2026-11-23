"""Forward kinematics for the Walter arms plus the placeholder gripper.

Each chain lists, for joint_1..joint_6, the joint origin in its parent link (xyz, rpy) and the
rotation axis, exactly as in the URDF macros, then the fixed transform from link_6 to tool0.
fk() returns the `tcp` frame in `base_link`. Used by the safety filter's workspace check and the
pose tests; test_kinematics.py checks every chain against the expanded URDF.

  irb120 : urdf/irb120_3_58_macro.xacro (ros-industrial abb_irb120_support)
  fr3    : urdf/fairino_fr3_macro.xacro (FAIR-INNOVATION fairino3_v6.urdf)
"""

from __future__ import annotations

import numpy as np

X, Y, Z = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
HALF_PI = 1.5708  # the FR3 URDF rounds pi/2 to this; keep it so FK matches the description exactly

CHAINS = {
    "irb120": {
        "joints": (
            ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), Z),
            ((0.0, 0.0, 0.29), (0.0, 0.0, 0.0), Y),
            ((0.0, 0.0, 0.27), (0.0, 0.0, 0.0), Y),
            ((0.0, 0.0, 0.07), (0.0, 0.0, 0.0), X),
            ((0.302, 0.0, 0.0), (0.0, 0.0, 0.0), Y),
            ((0.072, 0.0, 0.0), (0.0, 0.0, 0.0), X),
        ),
        "tool0": ((0.0, 0.0, 0.0), (0.0, np.pi / 2, 0.0)),  # link_6 -> tool0
    },
    "fr3": {
        "joints": (
            ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), Z),
            ((0.0, 0.0, 0.14), (HALF_PI, 0.0, 0.0), Z),
            ((-0.28, 0.0, 0.0), (0.0, 0.0, 0.0), Z),
            ((-0.24001, 0.0, 0.0), (0.0, 0.0, 0.0), Z),
            ((0.0, 0.0, 0.102), (HALF_PI, 0.0, 0.0), Z),
            ((0.0, 0.0, 0.102), (-HALF_PI, 0.0, 0.0), Z),
        ),
        "tool0": ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    },
}
ROBOTS = tuple(CHAINS)
# tool0 (= gripper_body) -> tcp: 0.105 m along z, same placeholder gripper on both arms.
TCP_OFFSET = 0.105


def rot_axis(axis, angle: float) -> np.ndarray:
    """Rodrigues rotation about a unit axis."""
    k = np.asarray(axis, dtype=float)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * kx + (1 - np.cos(angle)) * kx @ kx


def rot_rpy(r: float, p: float, y: float) -> np.ndarray:
    """URDF convention: fixed-axis roll, pitch, yaw = Rz(y) Ry(p) Rx(r)."""
    return rot_axis(Z, y) @ rot_axis(Y, p) @ rot_axis(X, r)


def transform(xyz, rpy=(0.0, 0.0, 0.0)) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rot_rpy(*rpy)
    t[:3, 3] = xyz
    return t


def fk(q, robot: str = "irb120", tcp_offset: float = TCP_OFFSET) -> np.ndarray:
    """4x4 pose of the TCP in base_link for joint positions q (6,) in radians."""
    chain = CHAINS[robot]
    t = np.eye(4)
    for (xyz, rpy, axis), angle in zip(chain["joints"], q):
        t = t @ transform(xyz, rpy)
        t[:3, :3] = t[:3, :3] @ rot_axis(axis, angle)
    t = t @ transform(*chain["tool0"])
    return t @ transform((0.0, 0.0, tcp_offset))


def tcp_position(q, robot: str = "irb120") -> np.ndarray:
    return fk(q, robot)[:3, 3]
