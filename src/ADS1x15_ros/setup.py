import os
from glob import glob
from setuptools import find_packages, setup

package_name = "ads1x15_ros"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files.
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*launch.[pxy][yma]*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Damien LaRocque",
    maintainer_email="phicoltan@gmail.com",
    description="ROS 2 driver for the Adafruit ADS1x15 ADC Carrier Board",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ads1x15_node = ads1x15_ros.ads1x15_node:main",
        ],
    },
)
