#include "robot_control/base.h"
#include "robot_control/localization.h"
#include "robot_control/localizationEstimatePosition.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

class Localization{
public:
	Localization(ros::NodeHandle);
	virtual ~Localization();

	void odomHandler(const nav_msgs::Odometry::ConstPtr& odomData);
	bool estimatePosition(robot_control::localizationEstimatePosition::Request &req,
						  robot_control::localizationEstimatePosition::Response &res);

private:
	float posx, posy, posz, orientation;
	geometry_msgs::Pose pose;

	ros::NodeHandle node;
	ros::Subscriber odom;
	ros::Publisher localizationAdv;
	ros::ServiceServer advertiseEstimatePosition;

	robot_control::localization lMsg;
	robot_control::localizationEstimatePosition estimatePositionSrv;
};

Localization::Localization(ros::NodeHandle n){
	node = n;
	
	//Subscribe to GPS
	odom = node.subscribe(ODOMTOPIC, 100, &Localization::odomHandler, this);

	//Advertise Localization
	localizationAdv = n.advertise<robot_control::localization>(LOCALIZATIONTOPIC, 100);

	//Advertise Estimate Position
	advertiseEstimatePosition = node.advertiseService(ESTIMATEPOSITIONSERVICE, &Localization::estimatePosition, this);

}