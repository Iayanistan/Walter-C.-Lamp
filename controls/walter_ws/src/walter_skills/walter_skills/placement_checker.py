"""Answers "is the equipment where the taught poses expect it?" from AprilTags.

apriltag_ros publishes a TF frame per detected tag (named by `tag_frame_format`). For a station
the checker looks up its tag relative to the reference tag on the bench, takes the median of a few
samples, and compares it with the expected pose in stations.yaml.

    ros2 run walter_skills placement_checker
    ros2 service call /placement_checker/check walter_interfaces/srv/CheckPlacement "{station: hotplate}"
"""

from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from walter_interfaces.srv import CheckPlacement

from .cli import stations_path
from .placement import judge
from .tags import sample_relative, tag_frame


class PlacementChecker(Node):
    def __init__(self):
        super().__init__("placement_checker")
        self.declare_parameter("stations_file", str(stations_path()))
        self.declare_parameter("tag_frame_format", "tag_{id}")
        self.declare_parameter("max_age_s", 0.5)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_service(CheckPlacement, "~/check", self._check, callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f"stations from {self.get_parameter('stations_file').value}")

    def _cell(self):
        # Re-read on every call so a freshly taught station is used without a restart.
        return yaml.safe_load(Path(self.get_parameter("stations_file").value).expanduser().read_text())

    def _check(self, req, res):
        cell = self._cell()
        st = cell["stations"].get(req.station)
        if st is None or "tag" not in st:
            res.ok, res.tag_seen = False, False
            res.message = f"station '{req.station}' has no tag in stations.yaml"
            return res
        fmt = self.get_parameter("tag_frame_format").value
        tol = {**cell.get("default_tolerance", {}), **st["tag"].get("tolerance", {})}
        measured = sample_relative(self, self.buffer, tag_frame(fmt, cell["reference_tag"]),
                                   tag_frame(fmt, st["tag"]["id"]),
                                   max_age_s=float(self.get_parameter("max_age_s").value))
        if measured is None:
            res.ok, res.tag_seen = False, False
            res.message = (f"tag {st['tag']['id']} ({req.station}) or reference tag {cell['reference_tag']} "
                           "not visible: equipment missing, blocked or knocked over")
        else:
            res.tag_seen = True
            res.ok, res.error_mm, res.error_deg, res.message = judge(
                measured, st["tag"]["expected"], float(tol.get("mm", 3.0)), float(tol.get("deg", 3.0)))
        self.get_logger().info(f"{req.station}: {res.message}")
        return res


def main():
    rclpy.init()
    node = PlacementChecker()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
