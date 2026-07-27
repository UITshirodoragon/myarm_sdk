from glob import glob
from setuptools import setup

package_name = "myarm_joint_state_demo"

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
    description="Minimal animated joint-state demo for MyArm M750.",
    license="MIT",
    entry_points={
        "console_scripts": ["joint_state_demo = myarm_joint_state_demo.publisher:main"],
    },
)
