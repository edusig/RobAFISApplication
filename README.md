#RobAFISApplication
Stock transporting robotic application using two pioneers

##Installation

###Install Ubuntu

Ros Indigo supports Ubuntu [Saucy (13.10)]() and [Trusty (14.04)](http://releases.ubuntu.com/14.04/).

For more supported OSs check [Ros Wiki - Indigo Installation](http://wiki.ros.org/indigo/Installation)

###Install ROS
Follow the installation process on ROS Wiki, 
[Indigo installation on Ubuntu](http://wiki.ros.org/indigo/Installation/Ubuntu).

###Create a Workspace

Follow the first tutorial from ros wiki tutorials.

[Installing and Configuring the Envrionment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

###Clone RobAFISApplication

Clone this repository anywhere, then move the src to your workspace src and the product model to your gazebo folder

```
git clone https://github.com/edusig/RobAFISApplication.git
cd RobAFISApplication
mv src ~/catkin_ws/src
cp product ~/.gazebo/product
```

###Run catkin_make once
```
cd ~/catkin_ws
catkin_make
```

##Usage

To execute everything we will need three terminals. One will run rosore, the second will run most nodes and gazebo simulator and the third will run the operation node that sends commands to the application.

```
roscore
```

Just run this while inside your workspace (~/catkin_ws/).

```
roslaunch src/robot_control/launch/master.launch
```

Then in another terminal.

```
rosrun robot_control operation
```
