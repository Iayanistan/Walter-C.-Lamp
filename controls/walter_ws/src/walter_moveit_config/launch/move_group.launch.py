"""MoveIt move_group (+ optional RViz) for the Walter cell. Included by walter_bringup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _nodes(context):
    robot = LaunchConfiguration("robot").perform(context)
    hardware = LaunchConfiguration("hardware").perform(context)
    suffix = "_fr3" if robot == "fr3" else ""
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    xacro_file = os.path.join(get_package_share_directory("walter_description"), "urdf", "walter_cell.urdf.xacro")
    config = (
        MoveItConfigsBuilder("walter", package_name="walter_moveit_config")
        .robot_description(file_path=xacro_file, mappings={"robot": robot, "hardware": hardware})
        .robot_description_semantic(file_path=f"config/walter{suffix}.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path=f"config/joint_limits{suffix}.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .planning_scene_monitor(publish_robot_description=False, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    move_group = Node(package="moveit_ros_move_group", executable="move_group", output="screen",
                      parameters=[config.to_dict(), {"use_sim_time": use_sim_time}])
    rviz = Node(package="rviz2", executable="rviz2", output="log",
                arguments=["-d", os.path.join(get_package_share_directory("walter_description"), "rviz", "walter.rviz")],
                parameters=[config.robot_description, config.robot_description_semantic,
                            config.robot_description_kinematics, config.planning_pipelines, config.joint_limits],
                condition=IfCondition(LaunchConfiguration("launch_rviz")))
    return [move_group, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="irb120"),
        DeclareLaunchArgument("hardware", default_value="mock"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        OpaqueFunction(function=_nodes),
    ])
