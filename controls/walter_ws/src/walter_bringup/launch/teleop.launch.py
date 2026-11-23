"""Gamepad teleoperation through MoveIt Servo. Start bringup.launch.py first.

    ros2 launch walter_bringup teleop.launch.py
    ros2 launch walter_bringup teleop.launch.py robot:=fr3 hardware:=fairino
    ros2 launch walter_bringup teleop.launch.py instruction:="push the hot plate back onto its marks"

Left stick: move in the bench plane. Right stick: up/down and turn the wrist. D-pad: tilt.
RB/LB: open/close the gripper. START: begin recording an episode. BACK: save it. B: discard it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _nodes(context):
    robot = LaunchConfiguration("robot").perform(context)
    hardware = LaunchConfiguration("hardware").perform(context)
    suffix = "_fr3" if robot == "fr3" else ""
    use_sim_time = hardware == "isaac"
    xacro_file = os.path.join(get_package_share_directory("walter_description"), "urdf", "walter_cell.urdf.xacro")
    config = (
        MoveItConfigsBuilder("walter", package_name="walter_moveit_config")
        .robot_description(file_path=xacro_file, mappings={"robot": robot, "hardware": hardware})
        .robot_description_semantic(file_path=f"config/walter{suffix}.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path=f"config/joint_limits{suffix}.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
    servo = {"moveit_servo": ParameterBuilder("walter_moveit_config").yaml("config/servo.yaml").to_dict()}
    return [
        Node(package="moveit_servo", executable="servo_node", name="servo_node", output="screen",
             parameters=[servo, {"update_period": 0.01, "planning_group_name": "arm", "use_sim_time": use_sim_time},
                         config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics, config.joint_limits]),
        Node(package="joy", executable="joy_node", parameters=[{"deadzone": 0.1, "autorepeat_rate": 50.0}]),
        Node(package="walter_control", executable="gamepad_teleop", output="screen",
             # The FR3 base is turned 180 deg on the bench, so steer in the bench frame to keep the
             # sticks pointing the same way as on the IRB 120.
             parameters=[{"instruction": LaunchConfiguration("instruction").perform(context),
                          "frame": "bench" if robot == "fr3" else "base_link",
                          "use_sim_time": use_sim_time}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="irb120", choices=["irb120", "fr3"]),
        DeclareLaunchArgument("hardware", default_value="mock", choices=["mock", "isaac", "abb", "fairino"]),
        DeclareLaunchArgument("instruction", default_value="",
                              description="language instruction stored with each recorded episode"),
        OpaqueFunction(function=_nodes),
    ])
