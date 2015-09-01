#RobAFISApplication
Stock transporting robotic application using two pioneers

##Installation

###Install Ubuntu

Ros Indigo supports Ubuntu [Saucy (13.10)]() and [Trusty (14.04)](http://releases.ubuntu.com/14.04/).

For more supported OSs check [Ros Wiki - Indigo Installation](http://wiki.ros.org/indigo/Installation)

###Install ROS
Follow the installation process on ROS Wiki, 
[Indigo installation on Ubuntu](http://wiki.ros.org/indigo/Installation/Ubuntu)

###Install other needed packages

Some packages needed to run the application

```
sudo apt-get install ros-indigo-urdf ros-indigo-tf ros-indigo-gazebo-plugins ros-indigo-control-msgs
```

###Install Gazebo

Install Gazebo 4, them its plugins.

```
wget -O /tmp/gazebo4_install.sh http://osrf-distributions.s3.amazonaws.com/gazebo/gazebo4_install.sh; sudo sh /tmp/gazebo4_install.sh

sudo apt-get install ros-indigo-gazebo4-plugins

gazebo
```

###Create a Workspace

Follow the first tutorial from ros wiki tutorials.

[Installing and Configuring the Envrionment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

Make sure the ownership of your workspace folder is attributed to the user that installed ROS when executing `catking_init_workspace`.

```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

###Clone RobAFISApplication

Clone this repository anywhere, then move the src to your workspace src and the product model to your gazebo folder

```
cd ~
git clone https://github.com/edusig/RobAFISApplication.git
cd RobAFISApplication
mv src/* ~/catkin_ws/src
mkdir -p ~/.gazebo/models
cp -r product ~/.gazebo/models/
cd ..
rm -rfv RobAFISApplication
```

###Run catkin_make once

Try compiling the program and verify id everything is working fine.

```
cd ~/catkin_ws
catkin_make
```

##Usage

To execute everything we will need three terminal tabs. One will run rosore, the second will run most nodes and gazebo simulator and the third will run the operation node that sends commands to the application.

```
roscore
```

Just run this while inside your workspace (~/catkin_ws/).

```
cd ~/catkin_ws
roslaunch src/robot_control/launch/master.launch
```

Then in another terminal.

```
cd ~/catkin_ws
rosrun robot_control operation
```