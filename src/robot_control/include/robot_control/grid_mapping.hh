#include "robot_control/base.h"
#include "sensor_msgs/LaserScan.h"
#include "robot_control/gridMappingCreateMap.h"
#include "robot_control/gridMappingGridToLocation.h"
#include "robot_control/gridMappingLocationToGrid.h"
#include <iostream>
#include <fstream>

class Mapper{
public:

	Mapper(ros::NodeHandle);
	virtual ~Mapper();

	void laserHandler(const sensor_msgs::LaserScan::ConstPtr& laserData);

	bool createMap(robot_control::gridMappingCreateMap::Request &req,
				   robot_control::gridMappingCreateMap::Response & res);
	bool gridToLocation(robot_control::gridMappingGridToLocation::Request &req,
				   		robot_control::gridMappingGridToLocation::Response & res);
	bool locationToGrid(robot_control::gridMappingLocationToGrid::Request &req,
				  		robot_control::gridMappingLocationToGrid::Response & res);

private:

	int cellsize, mapwidth, mapheight;
	unsigned int gi;
	bool active, success;
	double *distances;
	double lastx, lasty, lastorientation, orientation_fix;
	float posx, posy, posz, orientation, cmpx, cmpy;
	std::vector<float> map;

	ros::NodeHandle node;

	ros::Subscriber laser;

	ros::ServiceServer advertiseCreateMap;
	robot_control::gridMappingCreateMap createMapSrv;

	std::ofstream mapfile;

	bool setMapCell(int, int, int);
	
};