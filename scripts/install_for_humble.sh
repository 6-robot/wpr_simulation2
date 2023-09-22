#!/bin/bash
mkdir -p ~/.gazebo
mkdir -p ~/.gazebo/models
mkdir -p ~/.gazebo/models/wpr_simulation2
cp -r ../../wpr_simulation2/models ~/.gazebo/models/wpr_simulation2/models
cp -r ../../wpr_simulation2/meshes ~/.gazebo/models/wpr_simulation2/meshes
cp -r ../../wpr_simulation2/worlds ~/.gazebo/models/wpr_simulation2/worlds
sudo apt update -y
sudo apt install -y ros-humble-gazebo-ros-pkgs
sudo apt install -y ros-humble-gazebo-plugins
sudo apt install -y ros-humble-ros2-control
sudo apt install -y ros-humble-ros2-controllers
sudo apt install -y ros-humble-gazebo-ros2-control
sudo apt install -y ros-humble-slam-toolbox
sudo apt install -y ros-humble-teleop-twist-keyboard
sudo apt install -y ros-humble-navigation2
sudo apt install -y ros-humble-nav2-bringup
sudo apt install -y ros-humble-pcl-ros
sudo apt install -y python3-pip
yes | pip3 install face_recognition
