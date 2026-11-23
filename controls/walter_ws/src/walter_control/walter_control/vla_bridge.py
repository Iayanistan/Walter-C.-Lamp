"""ROS 2 <-> GR00T bridge.

Action server /walter/run_policy (walter_interfaces/action/RunPolicy). For each goal it:
  1. asks the supervisor for mode VLA,
  2. loops: latest camera images + joint state -> GR00T observation -> policy server ->
     action chunk -> first `execution_horizon` steps published on /walter/vla/chunk (arm, checked by
     the safety filter) and streamed to the gripper controller step by step,
  3. returns the supervisor to IDLE when the goal ends, is cancelled, or the cell halts.

The policy server runs in the Isaac-GR00T environment, possibly on another machine
(see controls/vla/README.md). This node only needs pyzmq + msgpack-numpy.
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory

from walter_interfaces.action import RunPolicy
from walter_interfaces.srv import SetMode

from .common import CHUNK_TOPIC, GRIPPER_TOPIC, LATCHED, MODE_TOPIC, JointStateCache, make_trajectory
from .gr00t_client import Gr00tClient
from .observation import build_observation, image_to_rgb, resize_nearest, split_action


class VlaBridge(Node):
    def __init__(self):
        super().__init__("vla_bridge")
        p = {name: self.declare_parameter(name, default).value for name, default in [
            ("host", "127.0.0.1"), ("port", 5555), ("timeout_ms", 2000),
            ("video_keys", ["scene", "wrist"]),
            ("image_topics", ["/scene_camera/image_raw", "/wrist_camera/image_raw"]),
            ("image_width", 320), ("image_height", 240),
            ("state_keys", ["single_arm", "gripper"]), ("action_keys", ["single_arm", "gripper"]),
            ("language_key", "annotation.human.task_description"),
            ("arm_joints", [f"joint_{i}" for i in range(1, 7)]), ("gripper_joint", "gripper_joint"),
            ("control_hz", 15.0), ("execution_horizon", 8), ("actions_are_deltas", False),
            ("default_max_duration", 60.0), ("max_image_age", 0.5),
        ]}
        self.p = p
        self.arm_joints = list(p["arm_joints"])
        self.joints = JointStateCache(self.arm_joints + [p["gripper_joint"]])
        self.images: dict[str, tuple[float, np.ndarray]] = {}
        self.mode = "IDLE"
        cb = ReentrantCallbackGroup()

        self.client = Gr00tClient(p["host"], int(p["port"]), int(p["timeout_ms"]))
        self.chunk_pub = self.create_publisher(JointTrajectory, CHUNK_TOPIC, 10)
        self.grip_pub = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
        self.create_subscription(JointState, "/joint_states", self.joints.update, 10, callback_group=cb)
        self.create_subscription(String, MODE_TOPIC, lambda m: setattr(self, "mode", m.data), LATCHED,
                                 callback_group=cb)
        for key, topic in zip(p["video_keys"], p["image_topics"]):
            self.create_subscription(Image, topic, lambda m, k=key: self._on_image(k, m), 2, callback_group=cb)
        self.set_mode = self.create_client(SetMode, "/supervisor/set_mode", callback_group=cb)
        self.server = ActionServer(self, RunPolicy, "/walter/run_policy", execute_callback=self._execute,
                                   goal_callback=lambda g: GoalResponse.ACCEPT,
                                   cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=cb)
        reachable = self.client.ping()
        self.get_logger().info(f"GR00T server {p['host']}:{p['port']} " + ("reachable" if reachable else "NOT reachable yet"))

    def _on_image(self, key: str, msg: Image):
        try:
            rgb = image_to_rgb(msg.encoding, msg.height, msg.width, msg.step, bytes(msg.data))
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return
        self.images[key] = (time.monotonic(), resize_nearest(rgb, self.p["image_width"], self.p["image_height"]))

    def _request_mode(self, mode: str) -> tuple[bool, str]:
        if not self.set_mode.wait_for_service(timeout_sec=2.0):
            return False, "supervisor not available"
        res = self.set_mode.call(SetMode.Request(mode=mode))
        return res.success, res.message

    def _observation(self, instruction: str):
        now = time.monotonic()
        images = {}
        for key in self.p["video_keys"]:
            if key not in self.images:
                return None, f"no image yet for camera '{key}'"
            stamp, img = self.images[key]
            if now - stamp > self.p["max_image_age"]:
                return None, f"camera '{key}' image is {now - stamp:.2f} s old"
            images[key] = img
        if not self.joints.ready():
            return None, "no joint states yet"
        arm = self.joints.get(self.arm_joints)
        grip = self.joints.get([self.p["gripper_joint"]])
        states = dict(zip(self.p["state_keys"], [arm, grip]))
        return build_observation(images, states, instruction, self.p["language_key"]), ""

    def _execute(self, goal_handle):
        goal = goal_handle.request
        max_duration = goal.max_duration or self.p["default_max_duration"]
        result = RunPolicy.Result()
        ok, msg = self._request_mode("VLA")
        if not ok:
            goal_handle.abort()
            result.message = f"could not enter VLA mode: {msg}"
            return result

        dt = 1.0 / self.p["control_hz"]
        arm_key, grip_key = (list(self.p["action_keys"]) + [None])[:2]
        feedback = RunPolicy.Feedback()
        start = time.monotonic()
        self.client.reset()
        try:
            while True:
                elapsed = time.monotonic() - start
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.message = "cancelled"
                    break
                if self.mode != "VLA":
                    goal_handle.abort()
                    result.message = f"mode changed to {self.mode}"
                    break
                if elapsed >= max_duration:
                    goal_handle.succeed()
                    result.success, result.message = True, f"ran for {elapsed:.1f} s"
                    break

                obs, why = self._observation(goal.instruction)
                if obs is None:
                    goal_handle.abort()
                    result.message = why
                    break
                t0 = time.monotonic()
                action, _ = self.client.get_action(obs)
                feedback.last_inference_ms = (time.monotonic() - t0) * 1e3

                arm, grip = split_action(action, arm_key, grip_key, self.joints.get(self.arm_joints),
                                         self.p["actions_are_deltas"])
                n = min(int(self.p["execution_horizon"]), arm.shape[0])
                times = dt * np.arange(1, n + 1)
                self.chunk_pub.publish(make_trajectory(self.arm_joints, arm[:n], times))
                # The safety filter may slow the arm down; the gripper follows the nominal timing.
                for k in range(n):
                    if grip is not None:
                        self.grip_pub.publish(Float64MultiArray(data=[float(grip[k])]))
                    time.sleep(dt)
                result.chunks_executed += 1
                feedback.chunks_executed = result.chunks_executed
                feedback.elapsed = time.monotonic() - start
                goal_handle.publish_feedback(feedback)
        except (TimeoutError, RuntimeError, KeyError) as e:
            goal_handle.abort()
            result.message = f"policy error: {e}"
        finally:
            if self.mode == "VLA":
                self._request_mode("IDLE")
        self.get_logger().info(f"run_policy '{goal.instruction}' ended: {result.message}")
        return result


def main():
    rclpy.init()
    node = VlaBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.client.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
