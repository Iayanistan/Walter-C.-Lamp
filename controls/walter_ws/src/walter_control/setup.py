from glob import glob

from setuptools import find_packages, setup

package_name = "walter_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Walter C. Lamp team",
    maintainer_email="noreply@example.com",
    description="Supervisor, safety filter, GR00T policy bridge and D4607 task runner for the Walter cell.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "supervisor = walter_control.supervisor:main",
            "safety_filter = walter_control.safety_filter:main",
            "vla_bridge = walter_control.vla_bridge:main",
            "task_runner = walter_control.task_runner:main",
            "episode_recorder = walter_control.episode_recorder:main",
            "fake_policy_server = walter_control.fake_policy_server:main",
            "test_pattern_camera = walter_control.pattern_camera:main",
            "gamepad_teleop = walter_control.gamepad_teleop:main",
            "io_gripper = walter_control.io_gripper:main",
            "fr_io_gripper = walter_control.fr_io_gripper:main",
        ],
    },
)
