"""Gate between the GR00T bridge and joint_trajectory_controller.

Subscribes to chunks on /walter/vla/chunk (JointTrajectory, arm joints, time_from_start spaced at
the policy rate), runs chunk_guard.check_chunk on each, and forwards the result to the controller
only while the supervisor reports mode VLA. MoveIt and the task runner do not pass through here.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from walter_interfaces.msg import SafetyStatus

from .chunk_guard import GuardConfig, check_chunk
from .common import CHUNK_TOPIC, JTC_TOPIC, LATCHED, MODE_TOPIC, JointStateCache, hold_trajectory, make_trajectory


class SafetyFilter(Node):
    def __init__(self):
        super().__init__("safety_filter")
        p = {name: self.declare_parameter(name, default).value for name, default in [
            ("joint_names", [f"joint_{i}" for i in range(1, 7)]),
            ("lower_limits", [-3.14] * 6), ("upper_limits", [3.14] * 6), ("joint_margin", 0.05),
            ("max_velocities", [1.0] * 6), ("velocity_scale", 0.25), ("max_start_jump", 0.15),
            ("workspace_min", [-1.0, -1.0, -1.0]), ("workspace_max", [1.0, 1.0, 1.0]),
            ("chunk_timeout", 1.0), ("robot", "irb120"),
        ]}
        self.joint_names = list(p["joint_names"])
        self.cfg = GuardConfig(
            lower=np.array(p["lower_limits"]), upper=np.array(p["upper_limits"]),
            max_velocity=np.array(p["max_velocities"]) * p["velocity_scale"],
            margin=p["joint_margin"], max_start_jump=p["max_start_jump"],
            workspace_min=np.array(p["workspace_min"]), workspace_max=np.array(p["workspace_max"]),
            robot=p["robot"],
        )
        self.chunk_timeout = p["chunk_timeout"]
        self.mode = "IDLE"
        self.joints = JointStateCache(self.joint_names)
        self.status = SafetyStatus()
        self._chunk_end = None  # time the last forwarded chunk finishes
        self._held = True

        self.out = self.create_publisher(JointTrajectory, JTC_TOPIC, 10)
        self.status_pub = self.create_publisher(SafetyStatus, "/walter/safety/status", 10)
        self.create_subscription(JointState, "/joint_states", self.joints.update, 10)
        self.create_subscription(String, MODE_TOPIC, self._on_mode, LATCHED)
        self.create_subscription(JointTrajectory, CHUNK_TOPIC, self._on_chunk, 10)
        self.create_timer(0.05, self._watchdog)
        self.create_timer(0.1, self._publish_status)

    def _on_mode(self, msg: String):
        self.mode = msg.data
        self.status.mode = msg.data

    def _reject(self, reason: str):
        self.status.chunks_rejected += 1
        self.status.last_reason = reason
        self.get_logger().warn(f"chunk rejected: {reason}", throttle_duration_sec=1.0)

    def _on_chunk(self, msg: JointTrajectory):
        if self.mode != "VLA":
            return self._reject(f"mode is {self.mode}, not VLA")
        if not self.joints.ready():
            return self._reject("no joint states yet")
        try:
            order = [list(msg.joint_names).index(n) for n in self.joint_names]
        except ValueError:
            return self._reject(f"chunk joints {list(msg.joint_names)} do not cover {self.joint_names}")
        if not msg.points:
            return self._reject("empty chunk")
        positions = np.array([[pt.positions[i] for i in order] for pt in msg.points])
        t = [pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9 for pt in msg.points]
        dt = t[0] if len(t) == 1 else float(np.median(np.diff([0.0] + t)))

        result = check_chunk(self.joints.get(), positions, dt, self.cfg)
        if not result.ok:
            self._reject(result.reason)
            return self._hold()
        self.out.publish(make_trajectory(self.joint_names, result.positions, result.times))
        self.status.chunks_forwarded += 1
        if result.retimed:
            self.status.chunks_retimed += 1
        if result.reason:
            self.status.last_reason = result.reason
        self._chunk_end = self.get_clock().now().nanoseconds * 1e-9 + float(result.times[-1])
        self._held = False

    def _hold(self):
        if self.joints.ready():
            self.out.publish(hold_trajectory(self.joint_names, self.joints.get()))
        self._held = True

    def _watchdog(self):
        # Policy went quiet in VLA mode: stop at the measured position rather than coast.
        if self.mode != "VLA" or self._held or self._chunk_end is None:
            return
        if self.get_clock().now().nanoseconds * 1e-9 > self._chunk_end + self.chunk_timeout:
            self.get_logger().warn("no new chunk from the policy; holding")
            self._hold()

    def _publish_status(self):
        self.status.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(self.status)


def main():
    rclpy.init()
    node = SafetyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
