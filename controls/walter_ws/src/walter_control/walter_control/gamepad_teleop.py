"""Gamepad -> MoveIt Servo twist commands, gripper, and episode recording buttons.

Button and axis numbers are for an Xbox-style pad on Linux's joy driver; change the AXES/BUTTONS
tables if yours differ (`ros2 topic echo /joy` shows which index moves).
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from walter_interfaces.srv import StartEpisode

from .common import GRIPPER_TOPIC

AXES = {"left_x": 0, "left_y": 1, "right_x": 3, "right_y": 4, "dpad_x": 6, "dpad_y": 7}
BUTTONS = {"b": 1, "lb": 4, "rb": 5, "back": 6, "start": 7}


class GamepadTeleop(Node):
    def __init__(self):
        super().__init__("gamepad_teleop")
        self.instruction = self.declare_parameter("instruction", "").value
        self.frame = self.declare_parameter("frame", "base_link").value
        self.grip_open = self.declare_parameter("gripper_open", 0.04).value
        self.grip_closed = self.declare_parameter("gripper_closed", 0.0).value
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.grip_pub = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
        self.switch = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        self.rec = {name: self.create_client(srv, f"/episode_recorder/{name}")
                    for name, srv in (("start", StartEpisode), ("stop", Trigger), ("discard", Trigger))}
        self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.create_timer(0.02, self._tick)
        self.joy = None
        self.prev_buttons = []
        self.twist_mode = False

    def _ensure_twist_mode(self):
        if self.twist_mode or not self.switch.service_is_ready():
            return self.twist_mode
        self.switch.call_async(ServoCommandType.Request(command_type=ServoCommandType.Request.TWIST))
        self.twist_mode = True
        self.get_logger().info("servo in twist mode")
        return True

    def _pressed(self, msg, name):
        i = BUTTONS[name]
        was = self.prev_buttons[i] if i < len(self.prev_buttons) else 0
        return i < len(msg.buttons) and msg.buttons[i] and not was

    def _on_joy(self, msg):
        self.joy = msg
        if self._pressed(msg, "rb"):
            self.grip_pub.publish(Float64MultiArray(data=[self.grip_open]))
        if self._pressed(msg, "lb"):
            self.grip_pub.publish(Float64MultiArray(data=[self.grip_closed]))
        if self._pressed(msg, "start"):
            self._record("start", StartEpisode.Request(task=self.instruction))
        if self._pressed(msg, "back"):
            self._record("stop", Trigger.Request())
        if self._pressed(msg, "b"):
            self._record("discard", Trigger.Request())
        self.prev_buttons = list(msg.buttons)

    def _record(self, name, request):
        if not self.rec[name].service_is_ready():
            self.get_logger().warn("episode_recorder is not running")
            return
        self.rec[name].call_async(request).add_done_callback(
            lambda f: self.get_logger().info(f"recorder {name}: {f.result().message}"))

    def _axis(self, name):
        i = AXES[name]
        return float(self.joy.axes[i]) if self.joy and i < len(self.joy.axes) else 0.0

    def _tick(self):
        if self.joy is None or not self._ensure_twist_mode():
            return
        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame
        t.twist.linear.x = self._axis("left_y")
        t.twist.linear.y = self._axis("left_x")
        t.twist.linear.z = self._axis("right_y")
        t.twist.angular.z = self._axis("right_x")
        t.twist.angular.y = self._axis("dpad_y")
        t.twist.angular.x = self._axis("dpad_x")
        self.twist_pub.publish(t)


def main():
    rclpy.init()
    node = GamepadTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
