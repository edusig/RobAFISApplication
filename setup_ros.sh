#!/bin/bash

echo "||======================================================================||"
echo "||         Setting up your ubuntu to use the RobAFISApplication         ||"
echo "||                                                                      ||"
echo "||        This may take a few minutes and need your confirmation        ||"
echo "||                     for installing some packages                     ||"
echo "||                                                                      ||"
echo "||======================================================================||"

#Adds ros source and accept its keys
echo "Adding ros source and accepting its server keys"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

#Updates ubuntu sources and install ros core
echo "Updating packages and upgrading system"
sudo apt-get update
sudo apt-get upgrade
echo "Installing ros core"
sudo apt-get install ros-indigo-ros-base -y

#Install other needed packages
echo "Installing other necessary packages"
sudo apt-get install git curl -y
sudo apt-get install ros-indigo-urdf ros-indigo-tf ros-indigo-gazebo6-plugins ros-indigo-control-msgs
#ros-indigo-gazeboX-plugins

#Initialize ros dependencies and updates them
echo "Initializing ros dependencies and updating them"
sudo rosdep init
rosdep update

#Install latest gazebo
echo "Installing latest gazebo"
cd ~
curl http://osrf-distributions.s3.amazonaws.com/gazebo/gazebo6_install.sh >> gazebo6_install.sh
sudo sh gazebo6_install.sh 

#Adds ros executables to the terminal
echo "Adding ros executabes to user bashrc"
sudo -s
source /opt/ros/indigo/setup.bash
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

#Creates a ROS workspace to compile with catkin
echo "Creating a ROS workspace named ROSWS and copiling catkin"
sudo mkdir -p ~/ROSWS/src
cd ~/ROSWS/src
catkin_init_workspace
cd ..
catkin_make

#Adds your workspace packages to be executable through ros on the terminal
echo "Adding workspace executables to user bashrc"
sudo -s
source ~/ROSWS/devel/setup.bash
echo "source ~/ROSWS/devel/setup.bash" >> ~/.bashrc

#Goes back to home folder and clones RobAFISApplication repository
echo "Cloning RobAFISApplication repository to home folder them copying its content to the respective places"
cd ~
git clone https://github.com/edusig/RobAFISApplication.git
cd RobAFISApplication
mv src/* ~/ROSWS/src
cp product ~/.gazebo/product

#Goes back to workspace main directory and compiles with catkin
echo "Compiling your repository with the new code"
cd ~/ROSWS
catkin_make

#Updates terminal bash
sudo -s
source ~/.bashrc