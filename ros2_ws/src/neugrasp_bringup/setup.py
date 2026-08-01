from glob import glob

from setuptools import setup


package_name = "neugrasp_bringup"


setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="myarm_sdk maintainers",
    maintainer_email="maintainer@example.com",
    description="Application launch and static scene frames for Neugrasp.",
    license="All rights reserved",
    entry_points={
        "console_scripts": [
            "neugrasp_static_scene_frames_node = "
            "neugrasp_bringup.static_scene_frames:main",
        ],
    },
)
