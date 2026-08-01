from setuptools import setup

package_name = "myarm_robot_driver"


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
    description="ROS 2 driver boundary for the stateful MyArm M750 robot arm.",
    license="All rights reserved",
    entry_points={
        "console_scripts": [
            "myarm_robot_driver_node = myarm_robot_driver.robot_driver_node:main",
        ],
    },
)
