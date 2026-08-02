from setuptools import setup


package_name = "myarm_neugrasp"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="myarm_sdk maintainers",
    maintainer_email="maintainer@example.com",
    description="Sequential fake-scan and replay runtime nodes for NeuGrasp.",
    license="All rights reserved",
    entry_points={
        "console_scripts": [
            "neugrasp_scan_node = myarm_neugrasp.scan_node:main",
            "neugrasp_replay_node = myarm_neugrasp.replay_node:main",
            "neugrasp_trial_node = myarm_neugrasp.trial_node:main",
        ],
    },
)
