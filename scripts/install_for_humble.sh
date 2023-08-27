#!/bin/bash
mkdir -p ~/.gazebo
mkdir -p ~/.gazebo/models
cp -r ../../wpr_simulation2 ~/.gazebo/models/
sudo apt install -y ros-humble-gazebo-ros-pkgs
sudo apt install -y ros-humble-gazebo-plugins
sudo apt install -y ros-humble-ros2-control
sudo apt install -y ros-humble-ros2-controllers
sudo apt install -y ros-humble-gazebo-ros2-control
sudo apt install -y ros-humble-slam-toolbox
sudo apt install -y ros-humble-teleop-twist-keyboard
sudo apt install -y ros-humble-navigation2
sudo apt install -y ros-humble-nav2-bringup
