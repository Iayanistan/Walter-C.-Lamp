"""Runs a task file (config/d4607.yaml) step by step and logs timing for each step.

Step types:
  move     scripted joint-space move through named poses (FollowJointTrajectory, mode SCRIPTED)
  gripper  open/close the gripper (0.0 closed .. 0.04 open)
  wait     timed wait, e.g. the 30 +/- 2 s boil in D4607 10.4
  shake    sinusoidal shake on one joint, e.g. 30 +/- 1 s in 10.5
  vla      hand the arm to GR00T with a language instruction (RunPolicy action)
  operator print a prompt and wait for `ros2 service call /task_runner/confirm std_srvs/srv/Trigger`
  check    ask walter_skills' placement_checker whether a station's equipment is where it belongs;
           if not, try the step's `recover` GR00T instruction, then fall back to the operator

A failed step marked `timed` (the boil and shake windows) cannot be redone, so the runner logs
`sample_void` for it: that flask is discarded instead of recovered.

Each run writes events.jsonl (same shape as the spill-detection PoC's log) under
~/.ros/walter_runs/<timestamp>/ so step timing can be checked against the D4607 tolerances.

    ros2 run walter_control task_runner --ros-args -p task_file:=/path/to/d4607.yaml
"""

from __future__ import annotations

import json
import os
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from walter_interfaces.action import RunPolicy
from walter_interfaces.srv import CheckPlacement, SetMode

from .common import ARM_JOINTS, GRIPPER_TOPIC, JTC_ACTION, LATCHED, MODE_TOPIC, JointStateCache, make_trajectory
from .shake import shake_trajectory


class StepFailed(Exception):
    pass


class TaskRunner(Node):
    def __init__(self):
        super().__init__("task_runner")
        default = os.path.join(get_package_share_directory("walter_control"), "config", "d4607.yaml")
        self.declare_parameter("task_file", default)
        self.declare_parameter("speed_scale", 1.0)  # stretches scripted move durations by 1/speed_scale
        cb = ReentrantCallbackGroup()
        self.joints = JointStateCache(ARM_JOINTS)
        self.mode = "IDLE"
        self._confirm = threading.Event()

        self.create_subscription(JointState, "/joint_states", self.joints.update, 10, callback_group=cb)
        self.create_subscription(String, MODE_TOPIC, lambda m: setattr(self, "mode", m.data), LATCHED, callback_group=cb)
        self.grip_pub = self.create_publisher(Float64MultiArray, GRIPPER_TOPIC, 10)
        self.jtc = ActionClient(self, FollowJointTrajectory, JTC_ACTION, callback_group=cb)
        self.policy = ActionClient(self, RunPolicy, "/walter/run_policy", callback_group=cb)
        self.set_mode = self.create_client(SetMode, "/supervisor/set_mode", callback_group=cb)
        self.checker = self.create_client(CheckPlacement, "/placement_checker/check", callback_group=cb)
        self.create_service(Trigger, "~/confirm", self._on_confirm, callback_group=cb)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = Path.home() / ".ros" / "walter_runs" / stamp
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self._log_file = open(self.run_dir / "events.jsonl", "a", encoding="utf-8")
        self._t0 = time.monotonic()

    # bookkeeping --------------------------------------------------------------------------
    def log(self, event: str, **fields):
        rec = {"t": round(time.monotonic() - self._t0, 3), "wall": datetime.now().isoformat(timespec="seconds"),
               "event": event, **fields}
        self._log_file.write(json.dumps(rec) + "\n")
        self._log_file.flush()
        self.get_logger().info(f"{event} {fields}")

    def _on_confirm(self, req, res):
        self._confirm.set()
        res.success = True
        return res

    def _mode(self, mode: str):
        if not self.set_mode.wait_for_service(timeout_sec=5.0):
            raise StepFailed("supervisor not available")
        res = self.set_mode.call(SetMode.Request(mode=mode))
        if not res.success:
            raise StepFailed(f"supervisor refused {mode}: {res.message}")

    def _wait_joints(self):
        deadline = time.monotonic() + 5.0
        while not self.joints.ready():
            if time.monotonic() > deadline:
                raise StepFailed("no /joint_states")
            time.sleep(0.05)

    def _follow(self, positions, times):
        if not self.jtc.wait_for_server(timeout_sec=5.0):
            raise StepFailed("joint_trajectory_controller action server not available")
        goal = FollowJointTrajectory.Goal(trajectory=make_trajectory(ARM_JOINTS, positions, times))
        handle = self.jtc.send_goal_async(goal)
        while not handle.done():
            time.sleep(0.01)
        gh = handle.result()
        if not gh.accepted:
            raise StepFailed("trajectory goal rejected")
        res = gh.get_result_async()
        while not res.done():
            if self.mode == "HALTED":
                gh.cancel_goal_async()
                raise StepFailed("cell halted")
            time.sleep(0.01)
        code = res.result().result.error_code
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise StepFailed(f"trajectory failed with error code {code}: {res.result().result.error_string}")

    # step types ---------------------------------------------------------------------------
    def step_move(self, step, poses):
        self._wait_joints()
        names = step["through"] if isinstance(step["through"], list) else [step["through"]]
        seg = float(step.get("segment_s", 3.0)) / float(self.get_parameter("speed_scale").value)
        positions = [poses[n] for n in names]
        self._mode("SCRIPTED")
        try:
            self._follow(positions, seg * np.arange(1, len(positions) + 1))
        finally:
            if self.mode == "SCRIPTED":
                self._mode("IDLE")

    def step_gripper(self, step, poses):
        self.grip_pub.publish(Float64MultiArray(data=[float(step["position"])]))
        time.sleep(float(step.get("settle_s", 1.0)))

    def step_wait(self, step, poses):
        end = time.monotonic() + float(step["seconds"])
        while time.monotonic() < end:
            if self.mode == "HALTED":
                raise StepFailed("cell halted")
            time.sleep(0.05)

    def step_shake(self, step, poses):
        self._wait_joints()
        q0 = self.joints.get()
        t, q = shake_trajectory(q0, ARM_JOINTS.index(step["joint"]), np.radians(step["amplitude_deg"]),
                                step["frequency_hz"], step["duration_s"], step.get("ramp_s", 0.5))
        self._mode("SCRIPTED")
        try:
            self._follow(q, t)
        finally:
            if self.mode == "SCRIPTED":
                self._mode("IDLE")

    def step_vla(self, step, poses):
        if not self.policy.wait_for_server(timeout_sec=5.0):
            raise StepFailed("vla_bridge not running (ros2 launch walter_bringup vla.launch.py)")
        goal = RunPolicy.Goal(instruction=step["instruction"], max_duration=float(step.get("max_duration", 0.0)))
        handle = self.policy.send_goal_async(goal)
        while not handle.done():
            time.sleep(0.01)
        res = handle.result().get_result_async()
        while not res.done():
            time.sleep(0.05)
        result = res.result().result
        if not result.success:
            raise StepFailed(f"policy: {result.message}")

    def step_operator(self, step, poses):
        self._confirm.clear()
        self.log("operator_prompt", prompt=step["prompt"])
        self.get_logger().warn(f"OPERATOR: {step['prompt']}  ->  ros2 service call /task_runner/confirm std_srvs/srv/Trigger")
        while not self._confirm.wait(timeout=0.2):
            if self.mode == "HALTED":
                raise StepFailed("cell halted")

    def _placement(self, station):
        if not self.checker.wait_for_service(timeout_sec=5.0):
            raise StepFailed("placement_checker not running (ros2 run walter_skills placement_checker)")
        return self.checker.call(CheckPlacement.Request(station=station))

    def step_check(self, step, poses):
        """Recovery ladder: check -> GR00T recovery (if given) -> operator -> give up."""
        station = step["station"]
        res = self._placement(station)
        if res.ok:
            return
        self.log("placement_failed", station=station, reason=res.message, error_mm=round(res.error_mm, 1))
        for attempt in range(int(step.get("retries", 1)) if step.get("recover") else 0):
            self.log("recovery_started", station=station, instruction=step["recover"], attempt=attempt + 1)
            try:
                self.step_vla({"instruction": step["recover"], "max_duration": step.get("recover_s", 30.0)}, poses)
            except StepFailed as e:
                self.log("recovery_failed", station=station, reason=str(e))
                if self.mode == "HALTED":
                    raise
            res = self._placement(station)
            if res.ok:
                self.log("recovered", station=station, by="vla")
                return
        if step.get("operator_fallback", True):
            self.step_operator({"prompt": f"Put the {station} equipment back on its marks ({res.message})"}, poses)
            res = self._placement(station)
            if res.ok:
                self.log("recovered", station=station, by="operator")
                return
        raise StepFailed(f"{station}: {res.message}")

    # main loop ----------------------------------------------------------------------------
    def run(self):
        path = self.get_parameter("task_file").value
        task = yaml.safe_load(Path(path).read_text())
        poses = {k: np.array(v, dtype=float) for k, v in task.get("poses", {}).items()}
        self.log("task_started", task=task.get("name", path), file=path)
        for i, step in enumerate(task["steps"]):
            kind = step["type"]
            label = step.get("label", f"{i}:{kind}")
            self.log("step_started", step=label, type=kind, spec=step.get("spec", ""))
            t0 = time.monotonic()
            try:
                getattr(self, f"step_{kind}")(step, poses)
            except StepFailed as e:
                self.log("step_failed", step=label, reason=str(e), duration_s=round(time.monotonic() - t0, 3))
                if step.get("timed"):
                    self.log("sample_void", step=label, spec=step.get("spec", ""),
                             reason="timed D4607 window broken; discard this flask and start a fresh one")
                self.log("task_aborted", step=label)
                return False
            duration = time.monotonic() - t0
            fields = {"step": label, "duration_s": round(duration, 3)}
            if "tolerance_s" in step:
                target = float(step.get("seconds", step.get("duration_s", 0.0)))
                fields["within_tolerance"] = abs(duration - target) <= float(step["tolerance_s"])
            self.log("step_done", **fields)
        self.log("task_done")
        return True


def main():
    rclpy.init()
    node = TaskRunner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        ok = node.run()
        node.get_logger().info(f"task {'finished' if ok else 'aborted'}; log in {node.run_dir}")
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
