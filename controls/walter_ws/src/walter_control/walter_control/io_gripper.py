"""Drive a pneumatic (open/closed) gripper from a controller digital output, for hardware:=abb.

On the real cell the gripper controller in ros2_control is still the mock one, so every node keeps
publishing to /gripper_position_controller/commands as before. This node listens to the same topic
and sets the IRC5 output through abb_rws_client's set_io_signal service:

    ros2 launch abb_bringup abb_rws_client.launch.py robot_ip:=192.168.10.2
    ros2 run walter_control io_gripper --ros-args -p signal:=DO10_1

Commands below `close_below` (metres of finger opening) close the gripper; anything else opens it.
NOT yet run against a controller.
"""

from __future__ import annotations

import rclpy
from abb_robot_msgs.srv import SetIOSignal
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .common import GRIPPER_TOPIC


class IOGripper(Node):
    def __init__(self):
        super().__init__("io_gripper")
        self.signal = self.declare_parameter("signal", "DO10_1").value
        self.close_value = self.declare_parameter("close_value", "1").value
        self.open_value = self.declare_parameter("open_value", "0").value
        self.close_below = self.declare_parameter("close_below", 0.02).value
        self.client = self.create_client(SetIOSignal, "/rws_client/set_io_signal")
        self.create_subscription(Float64MultiArray, GRIPPER_TOPIC, self._on_command, 10)
        self.last = None

    def _on_command(self, msg):
        if not msg.data:
            return
        value = self.close_value if msg.data[0] < self.close_below else self.open_value
        if value == self.last:
            return
        if not self.client.service_is_ready():
            self.get_logger().error("rws_client not running: gripper command dropped")
            return
        self.last = value
        future = self.client.call_async(SetIOSignal.Request(signal=self.signal, value=value))
        future.add_done_callback(lambda f: self._done(f, value))

    def _done(self, future, value):
        res = future.result()
        if res.result_code != 1:  # abb_robot_msgs/ServiceResponses SUCCESS = 1
            self.last = None
            self.get_logger().error(f"set {self.signal}={value} failed: {res.message}")
        else:
            self.get_logger().info(f"{self.signal} = {value}")


def main():
    rclpy.init()
    node = IOGripper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
