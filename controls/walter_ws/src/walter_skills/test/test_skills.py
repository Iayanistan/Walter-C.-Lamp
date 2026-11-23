"""Skill compiler, station file and placement maths. Plain pytest, no ROS graph needed."""

import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from walter_control.kinematics import tcp_position
from walter_skills.placement import judge, median_pose, placement_error, quat_from_yaw, yaw_from_quat
from walter_skills.skills import SKILLS, Cell, ProcedureError, compile_procedure, describe_skills

HERE = Path(__file__).resolve().parent
STATIONS = HERE.parent / "config" / "stations.yaml"
PROCEDURE = HERE.parent / "config" / "procedures" / "d4607_flask.yaml"
BRINGUP = HERE.parent.parent / "walter_bringup" / "config"
# (stations file, safety file, robot) for each arm
ARMS = [("stations.yaml", "safety.yaml", "irb120"), ("stations_fr3.yaml", "safety_fr3.yaml", "fr3")]
RUNNER_TYPES = {"move", "gripper", "wait", "shake", "vla", "operator", "check"}


@pytest.fixture
def cell():
    return Cell.load(STATIONS)


@pytest.mark.parametrize("stations,safety_file,robot", ARMS)
def test_station_poses_inside_safety_box_and_limits(stations, safety_file, robot):
    cell = Cell.load(STATIONS.parent / stations)
    safety = yaml.safe_load((BRINGUP / safety_file).read_text())["safety_filter"]["ros__parameters"]
    assert safety.get("robot", "irb120") == robot
    lo = np.array(safety["lower_limits"]) + safety["joint_margin"]
    hi = np.array(safety["upper_limits"]) - safety["joint_margin"]
    for name, q in cell.all_poses().items():
        q = np.array(q)
        assert np.all(q >= lo) and np.all(q <= hi), name
        p = tcp_position(q, robot)
        assert np.all(p >= safety["workspace_min"]) and np.all(p <= safety["workspace_max"]), (name, p)


def test_fr3_stations_cover_the_same_names(cell):
    # A procedure written for the IRB 120 must compile unchanged on the FR3.
    fr3 = Cell.load(STATIONS.parent / "stations_fr3.yaml")
    assert {n: set(s["poses"]) for n, s in fr3.stations.items()} == \
        {n: set(s["poses"]) for n, s in cell.stations.items()}
    assert {n: s.get("tag") for n, s in fr3.stations.items()} == {n: s.get("tag") for n, s in cell.stations.items()}
    compile_procedure(yaml.safe_load(PROCEDURE.read_text()), fr3)


def test_d4607_procedure_compiles_to_runner_steps(cell):
    task = compile_procedure(yaml.safe_load(PROCEDURE.read_text()), cell)
    assert task["name"] == "d4607_flask_1"
    labels = [s["label"] for s in task["steps"]]
    assert len(labels) == len(set(labels))
    for step in task["steps"]:
        assert step["type"] in RUNNER_TYPES
        for pose in step.get("through", []):
            assert pose in task["poses"]
    checks = [s["station"] for s in task["steps"] if s["type"] == "check"]
    assert checks[:4] == ["funnel", "cool_pad", "rack_1", "hotplate"]  # setup checks, then pick source, place target
    assert any(s.get("recover") for s in task["steps"] if s["type"] == "check")
    assert {s["label"] for s in task["steps"] if s.get("timed")} == {"boil", "16_shake/2_shake"}


def test_pick_expands_in_order(cell):
    task = compile_procedure({"steps": [{"pick": {"station": "hotplate"}}]}, cell)
    assert [s["type"] for s in task["steps"]] == ["check", "gripper", "move", "gripper", "move"]
    assert task["steps"][2]["through"] == ["hotplate.above", "hotplate.grasp"]
    assert task["steps"][1]["position"] == cell.grip_open and task["steps"][3]["position"] == cell.grip_closed


def test_untagged_station_is_not_checked(cell):
    task = compile_procedure({"steps": [{"hold_under": {"station": "dispenser"}}]}, cell)
    assert [s["type"] for s in task["steps"]] == ["move"]


@pytest.mark.parametrize("steps, msg", [
    ([{"fly": {}}], "unknown skill"),
    ([{"pick": {"station": "oven"}}], "unknown station"),
    ([{"pick": {}}], "missing required parameter 'station'"),
    ([{"pick": {"station": "hotplate", "speed": 2}}], "unknown parameter"),
    ([{"wait": {"seconds": "thirty"}}], "should be float"),
    ([{"hold_under": {"station": "hotplate"}}], "no taught pose 'under'"),
    ([], "no steps"),
])
def test_mistakes_are_reported(cell, steps, msg):
    with pytest.raises(ProcedureError, match=msg):
        compile_procedure({"steps": steps}, cell)


def test_describe_lists_every_skill():
    names = {s["name"] for s in describe_skills()}
    assert names == set(SKILLS)
    pick = next(s for s in describe_skills() if s["name"] == "pick")
    station = next(p for p in pick["params"] if p["name"] == "station")
    assert station["required"] and station["station"]


def test_placement_maths():
    assert yaw_from_quat(*quat_from_yaw(0.7)) == pytest.approx(0.7)
    mm, deg = placement_error([0.303, 0.204, 179.0], [0.300, 0.200, -179.0])
    assert mm == pytest.approx(5.0) and deg == pytest.approx(2.0)
    ok, *_ = judge([0.301, 0.200, 1.0], [0.300, 0.200, 0.0], 3.0, 3.0)
    assert ok
    ok, _, _, msg = judge([0.310, 0.200, 0.0], [0.300, 0.200, 0.0], 3.0, 3.0)
    assert not ok and "10.0 mm" in msg
    assert median_pose([[0, 0, 179.0], [0, 0, -179.0], [5, 5, 178.0]])[2] == pytest.approx(179.0)
    assert math.isclose(median_pose([[1, 2, 0], [3, 4, 0], [2, 3, 0]])[0], 2)
