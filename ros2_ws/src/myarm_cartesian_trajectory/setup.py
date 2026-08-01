from glob import glob

from setuptools import setup


package_name = "myarm_cartesian_trajectory"


setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="myarm_sdk maintainers",
    maintainer_email="maintainer@example.com",
    description="Plan-only Cartesian trajectory action and RViz preview for MyArm M750.",
    license="All rights reserved",
    entry_points={
        "console_scripts": [
            "myarm_cartesian_trajectory_node = "
            "myarm_cartesian_trajectory.cartesian_trajectory_node:main",
            "myarm_trajectory_preview_player_node = "
            "myarm_cartesian_trajectory.trajectory_preview_player_node:main",
        ],
    },
)
