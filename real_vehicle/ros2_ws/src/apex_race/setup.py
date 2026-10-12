"""Package metadata.

The car does *not* build this package: `start_apex_pipeline.sh` runs the node as
`python3 -m apex_race.nodes.race_driver_node` with
`PYTHONPATH=/work/ros2_ws/src/apex_race`, exactly as it runs every apex_telemetry
node, and `APEX_SKIP_BUILD=1` stays. This file exists so the package can be
built and introspected on a workstation.
"""

from setuptools import find_packages, setup

package_name = "apex_race"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config",
         ["config/vehicle_real.yaml", "config/race_driver.yaml", "config/slam_toolbox_race.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="APEX Team",
    maintainer_email="todo@example.com",
    description="Race driver and plant identification of the real car.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "race_driver_node = apex_race.nodes.race_driver_node:main",
        ],
    },
)
