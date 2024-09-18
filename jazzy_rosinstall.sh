#!/usr/bin/env bash
# Source for all this: https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debs.html

# To run this file on Ubuntu type in the terminal
# source rosinstall.sh

# ------ Ensure the system is in UTF-8 local mode ------ 
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# ------ Enable Required Repositories ------
# Ensure that Ubuntu Universe Repo is enabled
sudo apt install software-properties-common
sudo add-apt-repository universe

# ROS 2 GPG key with apt
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repo to your source list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Add additional development tool. 
# This may not be necessary??? Better safe than sorry
sudo apt update && sudo apt install ros-dev-tools

# ------ Installing ROS 2 ------
sudo apt update
sudo apt upgrade

# Install desktop version. Could do base version which doesn't have GUIs
# but the robot also runs Ubuntu so it should be able to handle it
sudo apt install ros-jazzy-desktop -y
sudo apt install ros-jazzy-ros-base -y

# ------ Setup Environment ------
source /opt/ros/jazzy/setup.bash
# sudo apt install python3-colcon-common-extensions python3-venv ros-jazzy-joy-linux -y
sudo apt install python3-colcon-common-extensions python3-setuptools python3-serial ros-jazzy-joy-linux -y

# ====== Add All Commands on Terminal Start =======
echo "# ROS2 Jazzy Data:" >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
