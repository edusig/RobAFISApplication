#include "robot_control/path_planning.hh"

PathPlanning::PathPlanning(ros::NodeHandle n, int w, double p, double s){
	node = n;
	width = w;
	precision = p;
	scale = s;
	definedMap = false;
	advertiseDefinePath = node.advertiseService(PATHPLANNINGDEFINEPATH, &PathPlanning::defineGlobalPath, this);
}

PathPlanning::~PathPlanning(){}

bool PathPlanning::defineGlobalPath(robot_control::pathPlanningDefineGlobalPath::Request &req,
									robot_control::pathPlanningDefineGlobalPath::Response &res){

	std::list<ipos_t> path;
	std::vector<float> xpath, ypath, rxpath, rypath;
	ipos_t begin, end;
	dpos_t next;

	int i,j;

	
	begin.i = (int) req.beginx;
	begin.j = (int) req.beginy;

	end.i = (int) req.endx;
	end.j = (int) req.endy;

	if(!definedMap){
		std::vector<float> auxMap;
		auxMap = req.map;
	  	map.resize((int)auxMap.size());
	  	// Rotate clockwise
		for(i = 0; i < auxMap.size(); i++)
			map[(int) floor(i/width) + (i%width * width)] = auxMap[i];
    	ogmap = new OGMap(map, width, precision, scale);
		definedMap = true;
	}

    //ogmap->printOccupationGrid("/home/edusig/ROSWS/ogmap_novo.txt");

	path = ogmap->getPath(begin, end);

    std::list<ipos_t>::iterator it = path.begin();

    while(it != path.end()){
        next = ogmap->itod(*it);
        xpath.push_back((float)next.x);
        ypath.push_back((float)next.y);
	    it++;
	}

	//Filtering
	float ld, ldx, ldy, distance, distx, disty;
	bool erased[(int) xpath.size()];
	ld = -1;
	ldx = ldy = 0;
	
	for(i = 1; i < (int) xpath.size(); i++){
		erased[i] = false;

		distance = sqrt(pow(xpath[i] - xpath[i-1], 2) + pow(ypath[i] - ypath[i-1], 2));
		//Se X e Y juntos mudam na mesma proporção
		if(distance == ld){

			erased[i-1] = true;

			//Para evitar eliminar caminhos que mudam em angulo 90 (tipo L)
			distx = fabs(xpath[i] - xpath[i-1]);
			disty = fabs(ypath[i] - ypath[i-1]);
			//Se a distancia X muda, mas Y não muda
			if(distx - ldx < 0.1 && disty - ldy > 0.1){
				erased[i-1] = false;
				erased[i] = true;
			}

			//Se a distancia X não muda, mas Y muda
			if(distx - ldx > 0.1 && disty - ldy < 0.1){
				erased[i-1] = false;
				erased[i] = true;
			}

			ldx = distx;
			ldy = disty;
		}else{
			ldx = 0;
			ldy = 0;
		}
		ld = distance;
	}

	for(i = 0; i < (int) xpath.size(); i++){
		if(!erased[i] || i == xpath.size() - 1){
			rxpath.push_back(xpath[i]);
			rypath.push_back(ypath[i]);
			//ROS_INFO("PathPlanning: Added Point (%.2f, %.2f)", ypath[i], xpath[i]);
		}
	}

	res.xpath = rypath;
	res.ypath = rxpath;

	return true;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "PathPlanning");

	ros::NodeHandle node;

	PathPlanning *pathplanning = new PathPlanning(node, 100, 1.0, 10.0);

	ROS_INFO("PathPlanning Running!!!");

	ros::spin();

}