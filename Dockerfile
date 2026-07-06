FROM docker.io/library/ros:jazzy-ros-base

LABEL org.opencontainers.image.source https://github.com/dimasad/fr3_motion

RUN <<EOF
set -e

# Install tools and franka_ros2 build dependencies
apt-get update
apt-get install -y \
  curl \
  build-essential \
  cmake \
  git \
  libeigen3-dev \
  libfmt-dev \
  libpoco-dev \
  ros-${ROS_DISTRO}-pinocchio \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Install ros2_control packages
apt-get install -y \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers

# Install Moveit2 and moveit servo
apt-get install -y \
  ros-${ROS_DISTRO}-moveit \
  ros-${ROS_DISTRO}-moveit-servo

# Install ros packages
apt-get install -y \
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-pip \
  python3-vcstool

rm -rf /var/lib/apt/lists/*
EOF

# Install libfranka, franka_ros2 and franka_description
# Note: The versions need to be set according to your robot's system version
# https://franka.de/fr3-compatibility-matrix
# Adjust these parameters in the docker-compose.yml or in a .env file
ARG LIBFRANKA_VERSION=0.20.4
ARG FRANKA_ROS2_VERSION=v3.4.0
ARG FRANKA_DESCRIPTION_VERSION=2.8.0

SHELL ["/bin/bash", "-c"]

RUN <<EOF
set -eo pipefail

. /opt/ros/${ROS_DISTRO}/setup.bash

mkdir -p /tmp/franka_ros2
cd /tmp/franka_ros2

curl -L -O https://github.com/frankarobotics/libfranka/releases/download/${LIBFRANKA_VERSION}/libfranka_${LIBFRANKA_VERSION}_`lsb_release -cs`_amd64.deb
apt-get update
apt-get install -y ./libfranka_${LIBFRANKA_VERSION}_`lsb_release -cs`_amd64.deb

git clone --recursive https://github.com/frankarobotics/franka_ros2.git --branch ${FRANKA_ROS2_VERSION}
rm -rf franka_ros2/franka_gazebo

git clone --recursive https://github.com/frankarobotics/franka_description.git --branch ${FRANKA_DESCRIPTION_VERSION}

rosdep update
rosdep install --from-paths . --ignore-src -r -y
rm -rf /var/lib/apt/lists/*

colcon build --install-base /opt/ros/${ROS_DISTRO}/franka --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=off -DBUILD_TESTS=OFF

cd ..
rm -rf /tmp/franka_ros2

echo "source /opt/ros/${ROS_DISTRO}/franka/setup.bash" >> ~/.bashrc
echo "source /opt/ros/${ROS_DISTRO}/franka/setup.sh" >> ~/.profile
EOF
