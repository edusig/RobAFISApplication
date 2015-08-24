#include "robot_control/base.h"
#include <iostream>
#include <fstream>
#include "robot_control/mapInformationGetGridMap.h"
#include "robot_control/mapInformationUpdateGridMap.h"
#include "robot_control/mapInformationUpdateMapPosition.h"

class MapInformation{
public:
	MapInformation(ros::NodeHandle);
	virtual ~MapInformation();

	bool updateGridMap(robot_control::mapInformationUpdateGridMap::Request &req,
					   robot_control::mapInformationUpdateGridMap::Response &res);
	bool updateMapPosition(robot_control::mapInformationUpdateMapPosition::Request &req,
						   robot_control::mapInformationUpdateMapPosition::Response &res);
	bool getGridMap(robot_control::mapInformationGetGridMap::Request &req,
					robot_control::mapInformationGetGridMap::Response &res);

private:
	ros::NodeHandle node;
	std::vector<float> map;

	ros::ServiceServer advertiseUpdateGrid;
	ros::ServiceServer advertiseUpdatePosition;
	ros::ServiceServer advertiseGetGrid;

	bool updateMap(std::vector<float> newmap);
	void logMap();
};