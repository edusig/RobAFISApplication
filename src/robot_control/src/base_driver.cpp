#include "robot_control/base_driver.hh"

BaseDriver::BaseDriver(ros::NodeHandle n){
	node = n;
	velocity = node.advertise<geometry_msgs::Twist>(VELTOPIC, 100);
	advertisePower = node.advertiseService(DRIVEPOWERSERVICE, &BaseDriver::setDrivePower, this);
	advertiseRotation = node.advertiseService(DRIVEROTATIONSERVICE, &BaseDriver::setRotation, this);
}

bool BaseDriver::setRotation(robot_control::baseDriverSetRotation::Request &req, 
							 robot_control::baseDriverSetRotation::Response &res){

	vel.angular.z = req.angle;
	velocity.publish(vel);
	res.status = 1;
	//ROS_INFO("Drive Angle Set to %.2f", vel.angular.z);

	return true;
}

bool BaseDriver::setDrivePower(robot_control::baseDriverSetDrivePower::Request &req, 
							   robot_control::baseDriverSetDrivePower::Response &res){

	vel.linear.x = req.power;
	velocity.publish(vel);
	res.status = 1;
	//ROS_INFO("Drive Power Set to %.2f", vel.linear.x);

	return true;
}

//When destroyed stop the robot
BaseDriver::~BaseDriver(){
	vel.linear.x = 0;
	vel.angular.z = 0;
	velocity.publish(vel);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "BaseDriver");

	ros::NodeHandle node;

	BaseDriver *base_driver = new BaseDriver(node);

	ROS_INFO("BaseDriver Running!!!");

	ros::spin();

}
