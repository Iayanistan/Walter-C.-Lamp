from glob import glob

from setuptools import find_packages, setup

package_name = "walter_skills"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config/procedures", glob("config/procedures/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Walter C. Lamp team",
    maintainer_email="noreply@example.com",
    description="Skill library, station map, AprilTag placement checks and procedure compiler for the Walter cell.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "compile = walter_skills.cli:compile_main",
            "run = walter_skills.cli:run_main",
            "skills = walter_skills.cli:skills_main",
            "teach = walter_skills.teach:main",
            "placement_checker = walter_skills.placement_checker:main",
            "fake_tags = walter_skills.fake_tags:main",
        ],
    },
)
