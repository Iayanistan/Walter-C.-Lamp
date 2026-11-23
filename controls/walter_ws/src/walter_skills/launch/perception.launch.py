"""Scene camera -> rectified image -> AprilTag TF frames -> placement_checker.

    ros2 launch walter_skills perception.launch.py                            # USB camera on /dev/video0
    ros2 launch walter_skills perception.launch.py camera:=false              # images come from Isaac Sim
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tags = PathJoinSubstitution([FindPackageShare("walter_skills"), "config", "apriltag.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value="true", description="start usb_cam for the scene camera"),
        DeclareLaunchArgument("video_device", default_value="/dev/video0"),
        DeclareLaunchArgument("camera_info_url", default_value="file://" + os.path.expanduser("~/.ros/camera_info/scene_camera.yaml")),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="usb_cam", executable="usb_cam_node_exe", name="scene_camera", namespace="scene_camera",
             condition=IfCondition(LaunchConfiguration("camera")),
             parameters=[{"video_device": LaunchConfiguration("video_device"), "image_width": 1280,
                          "image_height": 720, "framerate": 15.0, "pixel_format": "mjpeg2rgb",
                          "camera_name": "scene_camera", "frame_id": "scene_camera_optical_frame",
                          "camera_info_url": LaunchConfiguration("camera_info_url")}]),
        Node(package="image_proc", executable="rectify_node", name="rectify", namespace="scene_camera",
             remappings=[("image", "image_raw")],
             parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]),
        Node(package="apriltag_ros", executable="apriltag_node", name="apriltag",
             remappings=[("image_rect", "/scene_camera/image_rect"),
                         ("camera_info", "/scene_camera/camera_info")],
             parameters=[tags, {"use_sim_time": LaunchConfiguration("use_sim_time")}]),
        Node(package="walter_skills", executable="placement_checker", output="both",
             parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]),
    ])
