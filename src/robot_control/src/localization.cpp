#include "robot_control/localization.hh"

Localization::~Localization(){}

void Localization::odomHandler(const nav_msgs::Odometry::ConstPtr& odomData){

	//Update position data
	pose = odomData->pose.pose;
	lMsg.posx = pose.position.x;
	lMsg.posy = pose.position.y;
	lMsg.posz = pose.position.z;
	lMsg.orientation = tf::getYaw(pose.orientation); //x,y ou z

	//Publish updated localization
	localizationAdv.publish(lMsg);
}

bool Localization::estimatePosition(
	robot_control::localizationEstimatePosition::Request &req,
	robot_control::localizationEstimatePosition::Response &res){

	res.posx = lMsg.posx;
	res.posy = lMsg.posy;
	res.posz = lMsg.posz;
	res.orientation = lMsg.orientation;
	return true;
}


int main(int argc, char **argv){

	ros::init(argc, argv, "Localization");

	ros::NodeHandle node;
	
	Localization *localization = new Localization(node);

	ROS_INFO("Localization Running!!!");

	ros::spin();
}
