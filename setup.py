from glob import glob

from setuptools import find_packages, setup

package_name = "smart_carrier_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/maps", glob("maps/*")),
    ],
    install_requires=["setuptools", "smbus2"],
    zip_safe=True,
    maintainer="alanlu01",
    maintainer_email="alanlu01@users.noreply.github.com",
    description="ROS2 integration for the Smart Carrier robot",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ina3221_node = smart_carrier_robot.ina3221_node:main",
            "dispatch_bridge_node = smart_carrier_robot.dispatch_bridge_node:main",
            "navigator_node = smart_carrier_robot.navigator_node:main",
            "location_validator_node = smart_carrier_robot.location_validator_node:main",
            "location_calibrator_node = smart_carrier_robot.location_calibrator_node:main",
        ],
    },
)
