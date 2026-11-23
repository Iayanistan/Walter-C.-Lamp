"""Stand-in for camera + apriltag_ros when there are no real tags (mock hardware, dry runs).

Publishes a TF frame for the reference tag and for every station tag at its expected pose, so
every placement check passes. To rehearse a recovery, misplace one station:

    ros2 run walter_skills fake_tags --ros-args -p misplace:=hotplate -p offset_mm:=12.0

The misplaced tag snaps back the first time the supervisor enters VLA mode, which stands in for
GR00T pushing the equipment back. `-p heal_on_vla:=false` keeps it wrong (tests the operator fallback).
"""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from walter_control.common import LATCHED, MODE_TOPIC

from .cli import stations_path
from .placement import quat_from_yaw
from .tags import tag_frame


class FakeTags(Node):
    def __init__(self):
        super().__init__("fake_tags")
        self.declare_parameter("stations_file", str(stations_path()))
        self.declare_parameter("camera_frame", "scene_camera_optical_frame")
        self.declare_parameter("tag_frame_format", "tag_{id}")
        self.declare_parameter("misplace", "")
        self.declare_parameter("offset_mm", 10.0)
        self.declare_parameter("heal_on_vla", True)
        self.cell = yaml.safe_load(Path(self.get_parameter("stations_file").value).expanduser().read_text())
        self.misplaced = self.get_parameter("misplace").value
        self.tf = TransformBroadcaster(self)
        self.create_subscription(String, MODE_TOPIC, self._on_mode, LATCHED)
        self.create_timer(0.1, self._tick)
        if self.misplaced:
            self.get_logger().warn(f"{self.misplaced} is misplaced by {self.get_parameter('offset_mm').value} mm")

    def _on_mode(self, msg):
        if msg.data == "VLA" and self.misplaced and self.get_parameter("heal_on_vla").value:
            self.get_logger().info(f"{self.misplaced} moved back into place")
            self.misplaced = ""

    def _send(self, parent, child, x, y, z, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
        q = quat_from_yaw(yaw)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf.sendTransform(t)

    def _tick(self):
        fmt = self.get_parameter("tag_frame_format").value
        ref = tag_frame(fmt, self.cell["reference_tag"])
        self._send(self.get_parameter("camera_frame").value, ref, 0.0, 0.0, 0.9, 0.0)
        for name, st in self.cell["stations"].items():
            if "tag" not in st:
                continue
            x, y, yaw_deg = st["tag"]["expected"]
            if name == self.misplaced:
                x += self.get_parameter("offset_mm").value / 1000.0
            self._send(ref, tag_frame(fmt, st["tag"]["id"]), x, y, 0.0, math.radians(yaw_deg))


def main():
    rclpy.init()
    node = FakeTags()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
