#!/bin/bash -e

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

sudo apt-get update

echo -e "${GREEN}==== Installing External ROS Packages ====${NC}"

sudo apt install ros-foxy-xacro -y
sudo apt install ros-foxy-joint-state-publisher-gui -y
sudo apt install ros-foxy-joint-state-publisher -y
sudo apt install ros-foxy-robot-state-publisher -y
sudo apt install ros-foxy-nav2-* -y
sudo apt install ros-foxy-slam-toolbox -y
sudo apt install ros-foxy-teleop-twist-keyboard -y
sudo apt install ros-foxy-gazebo-ros-pkgs 