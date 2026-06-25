# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
from setuptools import setup
import os

package_name = "aerial_robot_core"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Junichiro",
    author_email="j-sugihara@dragon.t.u-tokyo.ac.jp",
    maintainer="Johannes Kübel",
    maintainer_email="johannes.kubel@dragon.t.u-tokyo.ac.jp",
    description="ROS 2 package for aerial_robot_core",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # "Robot_interface_node = aerial_robot_core.robot_interface_node:main"
        ],
    },
)
