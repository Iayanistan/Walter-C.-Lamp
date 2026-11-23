"""Every taught pose in the task file must pass the same workspace check the safety filter uses."""

from pathlib import Path

import numpy as np
import yaml

from walter_control.kinematics import tcp_position

HERE = Path(__file__).resolve().parent
TASK = HERE.parent / "config" / "d4607.yaml"
SAFETY = HERE.parent.parent / "walter_bringup" / "config" / "safety.yaml"
STEP_TYPES = {"move", "gripper", "wait", "shake", "vla", "operator", "check"}


def test_poses_inside_safety_box_and_limits():
    task = yaml.safe_load(TASK.read_text())
    safety = yaml.safe_load(SAFETY.read_text())["safety_filter"]["ros__parameters"]
    lo = np.array(safety["lower_limits"]) + safety["joint_margin"]
    hi = np.array(safety["upper_limits"]) - safety["joint_margin"]
    for name, q in task["poses"].items():
        q = np.array(q)
        assert np.all(q >= lo) and np.all(q <= hi), name
        p = tcp_position(q)
        assert np.all(p >= safety["workspace_min"]) and np.all(p <= safety["workspace_max"]), (name, p)


def test_steps_are_known_and_reference_poses():
    task = yaml.safe_load(TASK.read_text())
    for step in task["steps"]:
        assert step["type"] in STEP_TYPES
        if step["type"] == "move":
            for name in step["through"]:
                assert name in task["poses"]
