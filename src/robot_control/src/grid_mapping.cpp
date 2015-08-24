#include "robot_control/grid_mapping.hh"

Mapper::Mapper(ros::NodeHandle n){
	node = n;
	gi = 0;
	distances = (double*) calloc(LASERSIZE, sizeof(double));

	//Subscribe to laser and localization topics
	laser = node.subscribe(LASERTOPIC, 100, &Mapper::laserHandler, this);
	advertiseCreateMap = node.advertiseService(MAPPINGCREATEGRID, &Mapper::createMap, this);

	lastx = lasty = lastorientation = 0.0f;
	orientation_fix = 0.0f;

}

Mapper::~Mapper(){
	free(distances);
}

void Mapper::laserHandler(const sensor_msgs::LaserScan::ConstPtr& laserData){
	if(laserData != NULL){
		for(int i = 0; i < LASERSIZE; i++)
			distances[i] = laserData->ranges[i];
	}
}

bool Mapper::createMap(robot_control::gridMappingCreateMap::Request& req,
					   robot_control::gridMappingCreateMap::Response& res){

	int i, r, mapx, mapy;
	double base_angle, rad, angle, //angulos
	newx, newy, radius[LASERSIZE], //posições no mapa
	xDifference, yDifference, orientationDifference,//diferença nas posições
	realorientation;//base position and orientation

	//Inicializa o mapa com os tamanhos pedidos
	if(map.empty()){
		cellsize = req.cellsize;
		mapwidth = (req.width/cellsize);
		mapheight = (req.height/cellsize);
		map = std::vector<float>(mapwidth * mapheight, 0.0f);
		for(i = 0; i < mapwidth*mapheight; i++){
			if(i < mapwidth || i % mapwidth == 0)
				map[i] = 1.0f;
		}
	}

	posx = req.posx;
	posy = req.posy;
	posz = req.posz;
	realorientation = req.orientation;
	orientation = req.orientation - 1.57;//Apenas pra ficar mais facil visualizar o arquivo do mapa

	//Adiciona a posicao do laser a posicao central do robo
	cmpx = posx * 100 + (25 * cos(realorientation));
	cmpy = posy * 100 + (25 * sin(realorientation));

	mapx = (int) floor(cmpx/cellsize);
	mapy = (int) floor(cmpy/cellsize);

	//Se a posição x e y e a orientação for muito próxima da ultima vez que a função for chamada
	//Os dados do sensor serão parecidos e irrelevantes
	xDifference = abs(lastx - mapx);
	yDifference = abs(lasty - mapy);
	orientationDifference = abs(orientation - lastorientation) * cellsize;

	lastx = mapx;
	lasty = mapy;
	lastorientation = orientation;

	if(xDifference < 0.1 && yDifference < 0.1 && orientationDifference < 0.1)
		return false;

	for(i = 0; i < LASERSIZE; i++)
		radius[i] = distances[i] * 100;

	//A cada 0,00490625 radiano = 1i, pois 3.14/640, 640 = laser rays
	for(i = 0; i < LASERSIZE; i+=5){
		if(distances[i] >= 9.9)
			continue;
		rad = 0.00490625 * i;
		angle = rad + orientation;
		r = radius[i];
		mapx = (int) floor( ( ( cmpx ) + ( r * cos( angle ) ) ) / cellsize);
		mapy = (int) floor( ( ( cmpy ) + ( r * sin( angle ) ) ) / cellsize);
		setMapCell(mapx, mapy, 1);
	}

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

	res.map = map;
	res.size = map.size();
	return true;
}

bool Mapper::gridToLocation(robot_control::gridMappingGridToLocation::Request &req,
					robot_control::gridMappingGridToLocation::Response & res){
	res.posx = (req.mapx) * cellsize / 100;
	res.posy = (req.mapy) * cellsize / 100;
	return true;
}

bool Mapper::locationToGrid(robot_control::gridMappingLocationToGrid::Request &req,
					robot_control::gridMappingLocationToGrid::Response & res){
	res.mapx = (req.posx * 100 / cellsize);
	res.mapy = (req.posy * 100 / cellsize);
	return true;
}

bool Mapper::setMapCell(int mapx, int mapy, int val){
	if(mapx >= mapwidth || mapy >= mapheight || mapx <= 0 || mapy <= 0)
		return false;
	int index = (mapy * mapwidth) + mapx;
	double current = map[index];
	map[index] = (val+current) / 2;
	return true;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "Mapper");

	ros::NodeHandle node;

	Mapper *mapping = new Mapper(node);

	ROS_INFO("Mapper Running!!!");

	ros::spin();

}
