"""Bring up the Walter cell.

    ros2 launch walter_bringup bringup.launch.py                         # mock hardware
    ros2 launch walter_bringup bringup.launch.py hardware:=isaac          # Isaac Sim (start the sim first)
    ros2 launch walter_bringup bringup.launch.py hardware:=abb rws_ip:=192.168.125.1
    ros2 launch walter_bringup bringup.launch.py robot:=fr3                # FR3 model, mock hardware
    ros2 launch walter_bringup bringup.launch.py robot:=fr3 hardware:=fairino controller_ip:=192.168.58.2

robot picks the arm model and its controllers.yaml / safety.yaml (the FR3 uses the *_fr3.yaml copies).
hardware:=abb only works with robot:=irb120, and hardware:=fairino only with robot:=fr3.

Starts robot_state_publisher, ros2_control, the controllers, MoveIt, the safety filter and the
supervisor. The GR00T bridge is separate (vla.launch.py) so the arm can run without a GPU.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    hardware = LaunchConfiguration("hardware")
    suffix = PythonExpression(["'_fr3' if '", robot, "' == 'fr3' else ''"])
    use_sim_time = PythonExpression(["'", hardware, "' == 'isaac'"])

    args = [
        DeclareLaunchArgument("robot", default_value="irb120", choices=["irb120", "fr3"]),
        DeclareLaunchArgument("hardware", default_value="mock", choices=["mock", "isaac", "abb", "fairino"]),
        DeclareLaunchArgument("controller_ip", default_value="192.168.58.2",
                              description="FR3 controller IP (hardware:=fairino)"),
        DeclareLaunchArgument("rws_ip", default_value="None", description="IRC5 / RobotStudio IP (hardware:=abb)"),
        DeclareLaunchArgument("rws_port", default_value="80"),
        DeclareLaunchArgument("configure_via_rws", default_value="true",
                              description="false on OmniCore/RobotWare 7, which abb_ros2 cannot query over RWS"),
        DeclareLaunchArgument("egm_port", default_value="6511", description="Must match the EGM UDP device on the controller"),
        DeclareLaunchArgument("launch_moveit", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
    ]

    robot_description = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare("walter_description"), "urdf", "walter_cell.urdf.xacro"]),
            " robot:=", robot,
            " hardware:=", hardware,
            " controller_ip:=", LaunchConfiguration("controller_ip"),
            " rws_ip:=", LaunchConfiguration("rws_ip"),
            " rws_port:=", LaunchConfiguration("rws_port"),
            " configure_via_rws:=", LaunchConfiguration("configure_via_rws"),
            " egm_port:=", LaunchConfiguration("egm_port"),
        ]),
        value_type=str,
    )
    controllers = PathJoinSubstitution([FindPackageShare("walter_bringup"), "config", ["controllers", suffix, ".yaml"]])
    safety = PathJoinSubstitution([FindPackageShare("walter_bringup"), "config", ["safety", suffix, ".yaml"]])

    def spawner(name, *extra):
        return Node(package="controller_manager", executable="spawner",
                    arguments=[name, "--controller-manager", "/controller_manager", *extra],
                    parameters=[{"use_sim_time": use_sim_time}])

    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="both",
             parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}]),
        # In Jazzy the controller manager reads the URDF from /robot_description.
        Node(package="controller_manager", executable="ros2_control_node", output="both",
             parameters=[controllers, {"use_sim_time": use_sim_time}]),
        spawner("joint_state_broadcaster"),
        spawner("joint_trajectory_controller"),
        spawner("gripper_position_controller"),
        Node(package="walter_control", executable="safety_filter", output="both",
             parameters=[safety, {"use_sim_time": use_sim_time}]),
        Node(package="walter_control", executable="supervisor", output="both",
             parameters=[{"use_sim_time": use_sim_time}]),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare("walter_moveit_config"), "launch", "move_group.launch.py"]),
            launch_arguments={"robot": robot, "hardware": hardware, "use_sim_time": use_sim_time,
                              "launch_rviz": LaunchConfiguration("launch_rviz")}.items(),
            condition=IfCondition(LaunchConfiguration("launch_moveit")),
        ),
    ]
    return LaunchDescription(args + nodes)
