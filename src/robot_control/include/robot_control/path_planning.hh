#include "robot_control/base.h"
#include "robot_control/pathPlanningDefineGlobalPath.h"
#include "OGMap.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <string>

class PathPlanning{
public:
	PathPlanning(ros::NodeHandle, int, double, double);
	virtual ~PathPlanning();

	bool defineGlobalPath(robot_control::pathPlanningDefineGlobalPath::Request &req,
						  robot_control::pathPlanningDefineGlobalPath::Response &res);
	void defineGlobalPath();

private:
	ros::NodeHandle node;
	std::vector<float> map;
	OGMap *ogmap;
	int mapwidth, mapheight, width;
	double precision, scale;
	bool definedMap;

	ros::ServiceServer advertiseDefinePath;

	void filterPath(std::list<ipos_t>);

};