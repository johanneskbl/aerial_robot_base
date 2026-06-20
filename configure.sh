#!/bin/bash
set -e

# Get VERSION_CODENAME from /etc/os-release
DISTRO=$(grep '^VERSION_CODENAME=' /etc/os-release | cut -d'=' -f2)

# Conditional branching based on distro_codename
if [ "$DISTRO" = "jammy" ]; then
    echo "This is Ubuntu ($DISTRO). Installing ROS 2 Humble..."

    # Configure ROS 2 apt repository
    sudo apt-get update
    sudo apt-get install -y curl software-properties-common
    sudo add-apt-repository universe
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb

    # Install ROS 2 Humble
    sudo apt-get update
    sudo apt-get install -y \
        ros-humble-desktop \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-ros-gz-sim \
        ros-humble-ros-gz-bridge
    source /opt/ros/humble/setup.bash

    rm -rf /var/lib/apt/lists/* /tmp/ros2-apt-source.deb

    # Initialize rosdep
    sudo apt install -y python3-rosdep python3-pip
    pip3 install -U rosdep
    rosdep init
    rosdep update

    # Install C++ development tools
    sudo apt install -y \
        gdb \
        clang-format

    # Install Python build and development tools
    pip3 install -r src/aerial_robot_base/py_requirements.txt

    # Install other packages
    sudo apt-get install -y python3-catkin-tools python3-vcstool python-is-python3
fi
