from glob import glob
from setuptools import find_packages, setup

package_name = "a6_y_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="a6_y_robot_team",
    maintainer_email="2470428938@qq.com",
    description="Leader-follower joint teleoperation tools for A6-Y.",
    license="BSD",
    entry_points={
        "console_scripts": [
            "leader_follower_teleop = a6_y_teleop.leader_follower_teleop:main",
            "episode_marker = a6_y_teleop.episode_marker:main",
            "check_teleop_topics = a6_y_teleop.check_teleop_topics:main",
        ],
    },
)
