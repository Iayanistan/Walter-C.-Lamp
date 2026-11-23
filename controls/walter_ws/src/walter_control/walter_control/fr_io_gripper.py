"""Drive an open/closed gripper from the FR3's tool digital output, for robot:=fr3 hardware:=fairino.

Same idea as io_gripper (IRC5): every node keeps publishing to /gripper_position_controller/commands
and this node turns those commands into a tool DO through FAIRINO's Python SDK:

    ros2 run walter_control fr_io_gripper --ros-args -p tool_do:=0

Needs FAIRINO's Python SDK (`from fairino import Robot`), installed as FAIRINO's SDK docs describe.
This opens a second SDK connection next to fairino_hardware's. FAIRINO users report trouble with two
RPC clients at once, so test it on the bench first (Part B4 of the FR3 tutorial). NOT yet run against
a controller.
"""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .common import GRIPPER_TOPIC


class FRIOGripper(Node):
    def __init__(self):
        super().__init__("fr_io_gripper")
        self.ip = self.declare_parameter("controller_ip", "192.168.58.2").value
        self.tool_do = self.declare_parameter("tool_do", 0).value         # tool output 0 or 1
        self.close_level = self.declare_parameter("close_level", 1).value  # DO level that closes
        self.close_below = self.declare_parameter("close_below", 0.02).value
        from fairino import Robot  # imported here so the rest of the package works without the SDK
        self.robot = Robot.RPC(self.ip)
        self.create_subscription(Float64MultiArray, GRIPPER_TOPIC, self._on_command, 10)
        self.last = None
        self.get_logger().info(f"tool DO{self.tool_do} on {self.ip}")

    def _on_command(self, msg):
        if not msg.data:
            return
        level = self.close_level if msg.data[0] < self.close_below else 1 - self.close_level
        if level == self.last:
            return
        err = self.robot.SetToolDO(self.tool_do, level, 0, 0)
        if err != 0:
            self.last = None
            self.get_logger().error(f"SetToolDO({self.tool_do}, {level}) failed with error code {err}")
        else:
            self.last = level
            self.get_logger().info(f"tool DO{self.tool_do} = {level}")


def main():
    rclpy.init()
    node = FRIOGripper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
