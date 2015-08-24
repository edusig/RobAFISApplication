#include "robot_control/map_information.hh"

MapInformation::MapInformation(ros::NodeHandle n){
	node = n;

	advertiseUpdateGrid = node.advertiseService(MAPINFOUPDATEGRID, &MapInformation::updateGridMap, this);
	advertiseUpdatePosition = node.advertiseService(MAPINFOUPDATEPOSITION, &MapInformation::updateMapPosition, this);
	advertiseGetGrid = node.advertiseService(MAPINFOGETGRID, &MapInformation::getGridMap, this);
}

MapInformation::~MapInformation(){}

bool MapInformation::updateGridMap(robot_control::mapInformationUpdateGridMap::Request &req,
								   robot_control::mapInformationUpdateGridMap::Response &res){
	res.success = updateMap(req.map);
	return res.success;
}

bool MapInformation::updateMapPosition(
	robot_control::mapInformationUpdateMapPosition::Request &req,
	robot_control::mapInformationUpdateMapPosition::Response &res){
	map[req.index] = req.value;
	res.success = true;
	return res.success;
}

bool MapInformation::getGridMap(robot_control::mapInformationGetGridMap::Request &req,
								robot_control::mapInformationGetGridMap::Response &res){
	res.map = map;
	logMap();
	return true;
}

bool MapInformation::updateMap(std::vector<float> newmap){

	if(map.size() == 0)
		map.resize(newmap.size(), 0.0f);

	if(map.size() != newmap.size())
		return false;

	//ROS_INFO("Map Information: got map update!");
	for(int i = 0; i < (int) map.size(); i++){
		if(map[i] <= 0.94 && newmap[i] > 0.94)
			map[i] = newmap[i];
		/*printf("%d", (int) (map[i] > 0.94));
		if(i % 100 == 99)
			printf("\n");*/
	}

	return true;
}


//Saves the map to a file
void MapInformation::logMap(){
	int i;
	std::ofstream mapfile;
	mapfile.open("../../map.txt");
	if(!mapfile.is_open()){
		ROS_INFO("não abriu o arquivo");
		return;
	}
	mapfile << "1\n"; //Map not finished
	//Remove products from the mapping
	for(i = 0; i < 5; i++){
		map[2348+i] = 0;
		map[2448+i] = 0;
		map[2548+i] = 0;
		map[2648+i] = 0;
		map[2748+i] = 0;
		map[7348+i] = 0;
		map[7448+i] = 0;
		map[7548+i] = 0;
		map[7648+i] = 0;
		map[7748+i] = 0;
	}
	for(i = 0; i < map.size(); i++){//rows
		mapfile << (int) (map[i]>0.94);
		//At the end of each row add a new line just for easy visualization
		if(i % 100 == 99)
			mapfile << "\n";
	}
	mapfile.close();
}

int main(int argc, char **argv){

	ros::init(argc, argv, "MapInformation");

	ros::NodeHandle node;

	MapInformation *mapinformation = new MapInformation(node);

	ROS_INFO("MapInformation Running!!!");

	ros::spin();

}