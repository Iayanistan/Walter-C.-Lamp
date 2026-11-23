"""The cell must expand to valid URDF for every arm and hardware back end."""

import os
import subprocess
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory

ARM_JOINTS = [f"joint_{i}" for i in range(1, 7)]


def expand(robot: str, hardware: str) -> ET.Element:
    xacro_file = os.path.join(get_package_share_directory("walter_description"), "urdf", "walter_cell.urdf.xacro")
    urdf = subprocess.run(["xacro", xacro_file, f"robot:={robot}", f"hardware:={hardware}", "rws_ip:=192.168.125.1"],
                          check=True, capture_output=True, text=True).stdout
    return ET.fromstring(urdf)


@pytest.mark.parametrize("robot,hardware,plugin,interfaces", [
    # abb_hardware_interface refuses joints without both position and velocity commands.
    ("irb120", "mock", "mock_components/GenericSystem", ["position", "velocity"]),
    ("irb120", "isaac", "topic_based_ros2_control/TopicBasedSystem", ["position", "velocity"]),
    ("irb120", "abb", "abb_hardware_interface/ABBSystemHardware", ["position", "velocity"]),
    # fairino_hardware exports position only, for commands and state.
    ("fr3", "mock", "mock_components/GenericSystem", ["position"]),
    ("fr3", "isaac", "topic_based_ros2_control/TopicBasedSystem", ["position"]),
    ("fr3", "fairino", "fairino_hardware/FairinoHardwareInterface", ["position"]),
])
def test_hardware_plugin(robot, hardware, plugin, interfaces):
    root = expand(robot, hardware)
    arm = next(rc for rc in root.iter("ros2_control") if rc.get("name") == "walter_arm")
    assert arm.find("hardware/plugin").text == plugin
    assert [j.get("name") for j in arm.findall("joint")] == ARM_JOINTS
    for joint in arm.findall("joint"):
        assert [c.get("name") for c in joint.findall("command_interface")] == interfaces
        assert [s.get("name") for s in joint.findall("state_interface")] == interfaces


@pytest.mark.parametrize("robot", ["irb120", "fr3"])
def test_tcp_chain_exists(robot):
    root = expand(robot, "mock")
    links = {link.get("name") for link in root.iter("link")}
    for name in ("world", "bench", "base_link", "tool0", "tcp", "wrist_camera_optical_frame", "scene_camera_optical_frame"):
        assert name in links
