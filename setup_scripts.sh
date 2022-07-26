#!/bin/bash -e

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

sudo apt-get update

echo -e "${GREEN}==== Installing External ROS Packages ====${NC}"

sudo apt install ros-$ROS_DISTRO-xacro -y
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui -y
sudo apt install ros-$ROS_DISTRO-joint-state-publisher -y
sudo apt install ros-$ROS_DISTRO-robot-state-publisher -y

sudo apt install ros-$ROS_DISTRO-nav2-* -y
sudo apt install ros-$ROS_DISTRO-slam-toolbox -y
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard -y
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs
