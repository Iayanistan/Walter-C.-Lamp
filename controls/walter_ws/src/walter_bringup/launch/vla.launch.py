"""Start the GR00T bridge. Needs bringup.launch.py running and a GR00T policy server reachable.

    ros2 launch walter_bringup vla.launch.py host:=10.0.0.42 port:=5555
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vla = PathJoinSubstitution([FindPackageShare("walter_bringup"), "config", "vla.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="5555"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="walter_control", executable="vla_bridge", output="both",
             parameters=[vla, {"host": LaunchConfiguration("host"),
                               "port": LaunchConfiguration("port"),
                               "use_sim_time": LaunchConfiguration("use_sim_time")}]),
    ])
