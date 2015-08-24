#include "robot_control/base.h"
#include "geometry_msgs/Twist.h"
#include "robot_control/baseDriverSetDrivePower.h"
#include "robot_control/baseDriverSetRotation.h"

class BaseDriver{
public:
	BaseDriver(ros::NodeHandle);

	bool setDrivePower(robot_control::baseDriverSetDrivePower::Request &req,
					   robot_control::baseDriverSetDrivePower::Response &res);
	bool setRotation(robot_control::baseDriverSetRotation::Request &req,
					 robot_control::baseDriverSetRotation::Response &res);

	virtual ~BaseDriver();
private:
	ros::NodeHandle node;
	ros::Publisher velocity;
	ros::ServiceServer advertisePower, advertiseRotation;

	robot_control::baseDriverSetDrivePower setPowerService;
	robot_control::baseDriverSetRotation setRotationService;

	geometry_msgs::Twist vel;
};