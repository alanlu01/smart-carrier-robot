from glob import glob

from setuptools import find_packages, setup

package_name = "smart_carrier_api"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "smbus2"],
    zip_safe=True,
    maintainer="alanlu01",
    maintainer_email="alanlu01@users.noreply.github.com",
    description="Cloud API to ROS 2 bridge for the Smart Carrier robot",
    license="MIT",
    entry_points={
        "console_scripts": [
            "api_bridge = smart_carrier_api.bridge_node:main",
        ],
    },
)
