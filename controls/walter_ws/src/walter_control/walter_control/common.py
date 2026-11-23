"""Small ROS helpers shared by the nodes."""

from __future__ import annotations

import numpy as np
from builtin_interfaces.msg import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [f"joint_{i}" for i in range(1, 7)]
JTC_TOPIC = "/joint_trajectory_controller/joint_trajectory"
JTC_ACTION = "/joint_trajectory_controller/follow_joint_trajectory"
GRIPPER_TOPIC = "/gripper_position_controller/commands"
MODE_TOPIC = "/walter/mode"
CHUNK_TOPIC = "/walter/vla/chunk"

# Late joiners (safety filter, bridge) must see the current mode immediately.
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def duration(seconds: float) -> Duration:
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int(round((seconds - sec) * 1e9)))


def make_trajectory(joint_names, positions, times) -> JointTrajectory:
    """Header stamp left at zero, so joint_trajectory_controller starts it immediately."""
    msg = JointTrajectory()
    msg.joint_names = list(joint_names)
    for q, t in zip(np.asarray(positions, dtype=float), times):
        msg.points.append(JointTrajectoryPoint(positions=q.tolist(), time_from_start=duration(float(t))))
    return msg


def hold_trajectory(joint_names, current) -> JointTrajectory:
    """Single point at the measured position: stops whatever the controller was executing."""
    return make_trajectory(joint_names, [current], [0.05])


class JointStateCache:
    """Keeps the latest position of the named joints from /joint_states."""

    def __init__(self, names):
        self.names = list(names)
        self._pos = {}
        self.stamp = None

    def update(self, msg):
        for n, p in zip(msg.name, msg.position):
            self._pos[n] = p
        self.stamp = msg.header.stamp

    def ready(self) -> bool:
        return all(n in self._pos for n in self.names)

    def get(self, names=None) -> np.ndarray:
        return np.array([self._pos[n] for n in (names or self.names)], dtype=float)
