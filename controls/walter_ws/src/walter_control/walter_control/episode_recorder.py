"""Records demonstrations straight into a GR00T LeRobot v2 dataset.

Works the same in Isaac Sim and on the real cell, whatever drives the arm (task runner, MoveIt,
teleop). Samples at `fps`: latest image per camera + joint state. The action for frame k is the
joint state at frame k+1 (next-state actions), which is what a position-controlled arm executes.

    ros2 run walter_control episode_recorder --ros-args -p dataset_dir:=~/walter_data/flask_to_hotplate
    ros2 service call /episode_recorder/start walter_interfaces/srv/StartEpisode "{task: 'place the flask on the hot plate'}"
    ros2 service call /episode_recorder/stop std_srvs/srv/Trigger       # saves the episode
    ros2 service call /episode_recorder/discard std_srvs/srv/Trigger    # drops it
"""

from __future__ import annotations

import os
import threading

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import Trigger

from walter_interfaces.srv import StartEpisode

from .common import ARM_JOINTS, JointStateCache
from .lerobot_writer import Layout, LeRobotWriter
from .observation import image_to_rgb, resize_nearest


class EpisodeRecorder(Node):
    def __init__(self):
        super().__init__("episode_recorder")
        p = {name: self.declare_parameter(name, default).value for name, default in [
            ("dataset_dir", "~/walter_data/default"), ("fps", 15.0),
            ("video_keys", ["scene", "wrist"]),
            ("image_topics", ["/scene_camera/image_raw", "/wrist_camera/image_raw"]),
            ("image_width", 320), ("image_height", 240), ("gripper_joint", "gripper_joint"),
            ("robot_type", "abb_irb120"),   # fairino_fr3 on the FR3; keep datasets from each arm apart
        ]}
        self.p = p
        self.joint_names = ARM_JOINTS + [p["gripper_joint"]]
        self.joints = JointStateCache(self.joint_names)
        self.latest: dict[str, np.ndarray] = {}
        self.writer = LeRobotWriter(os.path.expanduser(p["dataset_dir"]), p["fps"],
                                    Layout(cameras=tuple(p["video_keys"])), robot_type=p["robot_type"],
                                    joint_names=self.joint_names)
        self._lock = threading.Lock()
        self._task = None
        self._states, self._frames = [], {k: [] for k in p["video_keys"]}
        cb = ReentrantCallbackGroup()

        self.create_subscription(JointState, "/joint_states", self.joints.update, 10, callback_group=cb)
        for key, topic in zip(p["video_keys"], p["image_topics"]):
            self.create_subscription(Image, topic, lambda m, k=key: self._on_image(k, m), 2, callback_group=cb)
        self.create_service(StartEpisode, "~/start", self._start, callback_group=cb)
        self.create_service(Trigger, "~/stop", self._stop, callback_group=cb)
        self.create_service(Trigger, "~/discard", self._discard, callback_group=cb)
        self.create_timer(1.0 / p["fps"], self._sample, callback_group=cb)
        self.get_logger().info(f"dataset {self.writer.root} has {len(self.writer.episodes)} episodes")

    def _on_image(self, key, msg):
        try:
            rgb = image_to_rgb(msg.encoding, msg.height, msg.width, msg.step, bytes(msg.data))
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return
        self.latest[key] = resize_nearest(rgb, self.p["image_width"], self.p["image_height"])

    def _sample(self):
        with self._lock:
            if self._task is None or not self.joints.ready():
                return
            if any(k not in self.latest for k in self._frames):
                self.get_logger().warn("waiting for all cameras", throttle_duration_sec=2.0)
                return
            self._states.append(self.joints.get())
            for k in self._frames:
                self._frames[k].append(self.latest[k])

    def _reset(self):
        self._task, self._states = None, []
        self._frames = {k: [] for k in self.p["video_keys"]}

    def _start(self, req, res):
        with self._lock:
            if self._task is not None:
                res.success, res.message = False, f"already recording '{self._task}'"
                return res
            self._reset()
            self._task = req.task
        res.success, res.message = True, f"recording episode {len(self.writer.episodes)}: {req.task}"
        self.get_logger().info(res.message)
        return res

    def _stop(self, req, res):
        with self._lock:
            task, states, frames = self._task, np.array(self._states), self._frames
            self._reset()
        if task is None or len(states) < 2:
            res.success, res.message = False, "nothing recorded"
            return res
        actions = np.vstack([states[1:], states[-1:]])  # next-state actions
        ep = self.writer.write_episode(task, states, actions, frames)
        res.success, res.message = True, f"saved episode {ep} ({len(states)} frames, {len(states) / self.p['fps']:.1f} s)"
        self.get_logger().info(res.message)
        return res

    def _discard(self, req, res):
        with self._lock:
            n = len(self._states)
            self._reset()
        res.success, res.message = True, f"discarded {n} frames"
        return res


def main():
    rclpy.init()
    node = EpisodeRecorder()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
