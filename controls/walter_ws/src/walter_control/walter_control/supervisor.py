"""Owns the cell's mode and the halt latch.

Modes: IDLE -> SCRIPTED (MoveIt / task runner moves) or VLA (GR00T bridge) -> back to IDLE.
HALTED is latched: entered from ~/halt, a True on /walter/halt, or a spill event from the
spill-detection camera, and left only through ~/reset. On entering HALTED the supervisor sends a
hold trajectory so joint_trajectory_controller stops whatever it was doing.

This is a software stop for development. It does not replace the IRC5 e-stop chain.
"""

from __future__ import annotations

import json
import socket
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

from walter_interfaces.srv import SetMode

from .common import ARM_JOINTS, JTC_TOPIC, LATCHED, MODE_TOPIC, JointStateCache, hold_trajectory

MODES = ("IDLE", "SCRIPTED", "VLA", "HALTED")


class Supervisor(Node):
    def __init__(self):
        super().__init__("supervisor")
        # UDP port the spill-detection PoC sends its events to (0 disables). See docs/WORKFLOW.md.
        self.declare_parameter("spill_udp_port", 50555)
        self.mode = "IDLE"
        self.reason = ""
        self.joints = JointStateCache(ARM_JOINTS)
        self._lock = threading.Lock()

        self.mode_pub = self.create_publisher(String, MODE_TOPIC, LATCHED)
        self.hold_pub = self.create_publisher(JointTrajectory, JTC_TOPIC, 10)
        self.create_subscription(JointState, "/joint_states", self.joints.update, 10)
        self.create_subscription(Bool, "/walter/halt", self._on_halt_topic, 10)
        self.create_service(SetMode, "~/set_mode", self._on_set_mode)
        self.create_service(Trigger, "~/halt", lambda req, res: self._trigger(res, self.halt("halt service")))
        self.create_service(Trigger, "~/reset", self._on_reset)
        self.create_timer(1.0, self._publish_mode)
        self._publish_mode()

        port = self.get_parameter("spill_udp_port").value
        if port:
            threading.Thread(target=self._spill_listener, args=(port,), daemon=True).start()

    # mode changes -------------------------------------------------------------------------
    def _publish_mode(self):
        self.mode_pub.publish(String(data=self.mode))

    def _set(self, mode: str, reason: str = ""):
        with self._lock:
            if mode != self.mode:
                self.get_logger().info(f"mode {self.mode} -> {mode}" + (f" ({reason})" if reason else ""))
            self.mode, self.reason = mode, reason
        self._publish_mode()

    def halt(self, reason: str) -> bool:
        if self.joints.ready():
            self.hold_pub.publish(hold_trajectory(ARM_JOINTS, self.joints.get()))
        else:
            self.get_logger().warn("halt requested before joint states arrived; no hold sent")
        self._set("HALTED", reason)
        self.get_logger().error(f"HALTED: {reason}")
        return True

    def _on_set_mode(self, req, res):
        mode = req.mode.upper()
        if mode not in MODES or mode == "HALTED":
            res.success, res.message = False, f"unknown or reserved mode {req.mode!r}; use ~/halt to halt"
        elif self.mode == "HALTED":
            res.success, res.message = False, f"halted ({self.reason}); call ~/reset first"
        else:
            self._set(mode)
            res.success, res.message = True, ""
        res.mode = self.mode
        return res

    def _on_reset(self, req, res):
        if self.mode != "HALTED":
            res.success, res.message = True, f"not halted (mode {self.mode})"
        else:
            self._set("IDLE", "reset")
            res.success, res.message = True, "reset to IDLE"
        return res

    def _on_halt_topic(self, msg: Bool):
        if msg.data and self.mode != "HALTED":
            self.halt("/walter/halt")

    @staticmethod
    def _trigger(res, ok):
        res.success = ok
        return res

    # spill camera -------------------------------------------------------------------------
    def _spill_listener(self, port: int):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", port))
        self.get_logger().info(f"listening for spill events on udp/{port}")
        while rclpy.ok():
            data, addr = sock.recvfrom(65535)
            try:
                event = json.loads(data.decode())
            except ValueError:
                continue
            if event.get("event") == "spill_halt" and self.mode != "HALTED":
                self.halt(f"spill: {event.get('material', '?')} during {event.get('step', '?')} from {addr[0]}")


def main():
    rclpy.init()
    node = Supervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
