"""Teach a station: store the arm's current joints as a named pose, or the station's tag pose.

    ros2 run walter_skills teach --station hotplate --pose grasp    # jog the arm there first
    ros2 run walter_skills teach --station hotplate --tag           # equipment exactly where it belongs

Writes $WALTER_STATIONS if set, else ~/walter_cell/stations.yaml (copied from the packaged file on
first use). Keep a separate file for Isaac Sim, e.g. WALTER_STATIONS=~/walter_cell/stations_sim.yaml.
Comments in the file are not kept, so keep notes in the description fields.
"""

from __future__ import annotations

import argparse
import os
import shutil
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from walter_control.common import ARM_JOINTS, JointStateCache

from .tags import sample_relative, tag_frame, wait_for


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--station", required=True)
    what = ap.add_mutually_exclusive_group(required=True)
    what.add_argument("--pose", help="pose name to store, e.g. above, grasp, pour")
    what.add_argument("--tag", action="store_true", help="store the station tag's current pose as expected")
    ap.add_argument("--tag-id", type=int, help="tag id, when the station has no tag yet")
    ap.add_argument("--tag-frame-format", default="tag_{id}")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(argv)[1:] if argv is None else argv)

    target = Path(os.environ.get("WALTER_STATIONS", "~/walter_cell/stations.yaml")).expanduser()
    if not target.exists():
        target.parent.mkdir(parents=True, exist_ok=True)
        from ament_index_python.packages import get_package_share_directory
        packaged = Path(get_package_share_directory("walter_skills")) / "config"
        mine = Path.home() / "walter_cell" / "stations.yaml"
        if (packaged / target.name).exists():   # e.g. stations_fr3.yaml starts from the FR3 placeholders
            shutil.copy(packaged / target.name, target)
        elif mine.exists():
            shutil.copy(mine, target)
        else:
            shutil.copy(packaged / "stations.yaml", target)
        print(f"created {target}")
    cell = yaml.safe_load(target.read_text())
    st = cell["stations"].setdefault(args.station, {"description": args.station, "poses": {}})

    rclpy.init()
    node = Node("teach")
    try:
        if args.pose:
            joints = JointStateCache(ARM_JOINTS)
            node.create_subscription(JointState, "/joint_states", joints.update, 10)
            deadline = time.monotonic() + 5.0
            while not joints.ready() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not joints.ready():
                raise SystemExit("no /joint_states: is bringup running?")
            q = [round(float(v), 4) for v in joints.get()]
            st.setdefault("poses", {})[args.pose] = q
            print(f"{args.station}.{args.pose} = {q}")
        else:
            tag_id = args.tag_id if args.tag_id is not None else st.get("tag", {}).get("id")
            if tag_id is None:
                raise SystemExit("station has no tag yet: pass --tag-id")
            buf = Buffer()
            listener = TransformListener(buf, node, spin_thread=True)  # noqa: F841 - must stay alive
            ref, frame = (tag_frame(args.tag_frame_format, cell["reference_tag"]),
                          tag_frame(args.tag_frame_format, tag_id))
            wait_for(buf, ref, frame, timeout_s=5.0)
            pose = sample_relative(node, buf, ref, frame, n=15)
            if pose is None:
                raise SystemExit(f"tag {tag_id} or the reference tag is not visible")
            st["tag"] = {**st.get("tag", {}), "id": tag_id, "expected": [round(v, 4) for v in pose]}
            print(f"{args.station} tag {tag_id} expected = {st['tag']['expected']}")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    target.write_text(yaml.safe_dump(cell, sort_keys=False, default_flow_style=None, width=120))
    print(f"saved {target}")
