import os
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from walter_control.kinematics import ROBOTS, fk, rot_axis, tcp_position, transform


def test_home_pose_points_down():
    # Zero pose plus J5 = 90 deg: wrist centre at x = 0.302, z = 0.29 + 0.27 + 0.07 = 0.63, then the
    # 0.072 m flange offset and the 0.105 m gripper both hang straight down.
    q = [0, 0, 0, 0, np.pi / 2, 0]
    np.testing.assert_allclose(tcp_position(q), [0.302, 0.0, 0.63 - 0.072 - 0.105], atol=1e-9)
    np.testing.assert_allclose(fk(q)[:3, 2], [0, 0, -1], atol=1e-9)


def test_joint_1_rotates_about_z():
    q = [np.pi / 2, 0, 0, 0, np.pi / 2, 0]
    np.testing.assert_allclose(tcp_position(q), [0.0, 0.302, 0.453], atol=1e-9)


def test_shoulder_forward_lowers_wrist():
    # Positive J2 tilts the upper arm forward (towards +x) on the IRB 120.
    assert tcp_position([0, 0.5, 0, 0, np.pi / 2, 0])[0] > 0.302


def _urdf_fk(robot: str, q) -> np.ndarray:
    """base_link -> tcp straight from the expanded cell URDF, to catch a CHAINS entry that drifts."""
    from ament_index_python.packages import get_package_share_directory
    xacro_file = os.path.join(get_package_share_directory("walter_description"), "urdf", "walter_cell.urdf.xacro")
    root = ET.fromstring(subprocess.run(["xacro", xacro_file, f"robot:={robot}"], check=True,
                                        capture_output=True, text=True).stdout)
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}  # not the ros2_control <joint> tags
    chain, link = [], "tcp"
    while link != "base_link":
        chain.append(by_child[link])
        link = chain[-1].find("parent").get("link")
    angles = dict(zip([f"joint_{i}" for i in range(1, 7)], q))
    T = np.eye(4)
    for j in reversed(chain):
        o = j.find("origin")
        xyz = [float(v) for v in o.get("xyz", "0 0 0").split()] if o is not None else [0.0] * 3
        rpy = [float(v) for v in o.get("rpy", "0 0 0").split()] if o is not None else [0.0] * 3
        T = T @ transform(xyz, rpy)
        if j.get("type") == "revolute":
            R = np.eye(4)
            R[:3, :3] = rot_axis([float(v) for v in j.find("axis").get("xyz").split()], angles[j.get("name")])
            T = T @ R
    return T


@pytest.mark.parametrize("robot", ROBOTS)
def test_chain_matches_urdf(robot):
    pytest.importorskip("ament_index_python")
    rng = np.random.default_rng(1)
    for _ in range(5):
        q = rng.uniform(-1.2, 1.2, 6)
        np.testing.assert_allclose(fk(q, robot), _urdf_fk(robot, q), atol=1e-6)


def test_fr3_ready_pose_points_down():
    # Upper arm and forearm folded over the bench, J5 = -90 deg: the tool hangs straight down.
    q = [0, -1.5708, 1.5708, -1.5708, -1.5708, 0]
    np.testing.assert_allclose(fk(q, "fr3")[:3, 2], [0, 0, -1], atol=1e-4)
    assert tcp_position(q, "fr3")[0] < 0   # the FR3 works over its own -x side
