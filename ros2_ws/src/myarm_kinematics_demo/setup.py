from glob import glob

from setuptools import setup


package_name = "myarm_kinematics_demo"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="myarm_sdk maintainers",
    maintainer_email="maintainer@example.com",
    description="Pinocchio IK/FK and remote RViz demonstration for MyArm M750.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "kinematics_node = myarm_kinematics_demo.kinematics_node:main",
        ],
    },
)
