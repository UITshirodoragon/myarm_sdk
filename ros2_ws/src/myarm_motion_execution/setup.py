from setuptools import setup

package_name = "myarm_motion_execution"


setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="myarm_sdk maintainers",
    maintainer_email="maintainer@example.com",
    description="ROS 2 action boundary for MyArm joint motion execution.",
    license="All rights reserved",
    entry_points={
        "console_scripts": [
            "myarm_motion_execution_node = myarm_motion_execution.motion_execution_node:main",
        ],
    },
)
