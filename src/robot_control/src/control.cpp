#include "robot_control/control.hh"

Control::Control(ros::NodeHandle n): 
	fwClient(NAVFOLLOWWALL, true),
	ndtClient(NAVDRIVETO, true),
	ndClient(NAVDOCKING, true),
	meServer(n, CONTROLMAPENVIRONMENT, boost::bind(&Control::mapEnvironment, this, _1), false),
	dtServer(n, CONTROLDRIVETO, boost::bind(&Control::driveTo, this, _1), false),
	ppServer(n, CONTROLPICKPRODUCT, boost::bind(&Control::pickProduct, this, _1), false),
	rpServer(n, CONTROLRELEASEPRODUCT, boost::bind(&Control::releaseProduct, this, _1), false){
	node = n;
	pathDefined = false;
	navDriveToFinished = true;
	creatingMap = false;	
	backwards = false;

	navigationStopClient = node.serviceClient<robot_control::deliberativeNavigationStop>(NAVSTOPSERVICE);
	createGridMapClient = node.serviceClient<robot_control::gridMappingCreateMap>(MAPPINGCREATEGRID);
	defineGlobalPathClient = node.serviceClient<robot_control::pathPlanningDefineGlobalPath>(PATHPLANNINGDEFINEPATH);
	updateMapClient = node.serviceClient<robot_control::mapInformationUpdateGridMap>(MAPINFOUPDATEGRID);
	getMapClient = node.serviceClient<robot_control::mapInformationGetGridMap>(MAPINFOGETGRID);
	graspObjectClient =  node.serviceClient<robot_control::productManipulationGrasp>(PRODUCTMANIPULATIONGRASP);
	releaseObjectClient =  node.serviceClient<robot_control::productManipulationRelease>(PRODUCTMANIPULATIONRELEASE);

	localization = node.subscribe(LOCALIZATIONTOPIC, 100, &Control::localizationHandler, this);

	advertiseStop = node.advertiseService(CONTROLSTOPSERVICE, &Control::stop, this);
	advertiseReportState = node.advertiseService(CONTROLREPORTSTATE, &Control::reportState, this);
	advertiseReturnPathDistance = node.advertiseService(CONTROLRETURNPATHDISTANCE, &Control::returnPathDistance, this);

	cellsize = 10.0f;
	mapwidth = 1000.0f;
	mapheight = 1000.0f;

	meServer.start();
	dtServer.start();
	ppServer.start();
	rpServer.start();

}

Control::~Control(){}

/*
*
*
*	Product Collecting
*
*
*/
void Control::driveToAction(){

	active = true;
	success = false;
	navDriveToFinished = true;
	bool preempted = false;
	int dockingpoints = 1;

	//Get any updates on current map
	callGetMapInformation();

	//Call pathplanning for path to destination
	defineGlobalPathCall();
	int pathLenght = xpath.size();

	ROS_INFO("%sControl: path length = %d", KCYN, pathLenght);
	if(pathLenght == 0)
		active = false;

	//TODO: Only if the last two points are very close
	//like (5.0,5.0) e (5.5, 5.0)
	if(picking && pathLenght > 0){
		std::vector<float>::iterator itxa, itxb, itya, ityb;
		itxa = xpath.end()-1;
		itxb = xpath.end()-2;
		itya = ypath.end()-1;
		ityb = ypath.end()-2;
		float dist = sqrt(pow((*itxa) - (*itxb), 2) + pow((*itya) - (*ityb), 2));
		ROS_INFO("Control: last two points dist = %.2f", dist);
		if(dist <= 10)
			dockingpoints = 3;
	}

	ROS_INFO("%sControl: Conseguiu o caminho", KCYN);
	//Wait for Navigation DriveTo action server to connect
	ndtClient.waitForServer();

	while(active){

		//If finished navigating to point and not at the goal
		if(navDriveToFinished){

			//If there are points to navigate to send the next one
			if(xpath.size() > 0 && ypath.size() > 0){

				ndtGoal.backup = false;
				if(backwards && pathLenght <= xpath.size()+1){
					ndtGoal.backup = true;
					backwards = false;
					xpath.erase(xpath.begin());
					ypath.erase(ypath.begin());
					xpath.erase(xpath.begin());
					ypath.erase(ypath.begin());
				}

				ROS_INFO("%sControl: Getting first point from path list", KCYN);
				//Get first coordinates from path
				ndtGoal.posx = xpath.front()/100;
				ndtGoal.posy = ypath.front()/100;

				//Remove these coordinates from the path
				xpath.erase(xpath.begin());
				ypath.erase(ypath.begin());

				ndtGoal.docking = false;
				//If it is picking and the last point, set docking true for navigation
				if(picking && xpath.size() < dockingpoints)
					ndtGoal.docking = true;
				if(releasing && xpath.size() <= 1)
					backwards = true;

				ROS_INFO("%sControl: Calling navigation driveTo", KCYN);
				//Send coordinates to navigation
				ndtClient.sendGoal(ndtGoal,
					boost::bind(&Control::driveToFinish, this, _1, _2),
					navDriveToClient::SimpleActiveCallback(),
					boost::bind(&Control::driveToUpdateHandler, this, _1));

				navDriveToFinished = false;

			}else{
				active = false;
				success = true;
				break;
			}
		}

		if(picking)
			preempted = ppServer.isPreemptRequested();
		else if(releasing)
			preempted = rpServer.isPreemptRequested();
		else
			preempted = dtServer.isPreemptRequested();

		if(preempted || !ros::ok()){
			if(picking)
				ppServer.setPreempted();
			else if(releasing)
				rpServer.setPreempted();
			else
				dtServer.setPreempted();
			success = false;
			break;
		}
	}

	ROS_INFO("%sControl: Finished DriveTo", KCYN);

}

void Control::driveToUpdateHandler(const robot_control::navDriveToFeedbackConstPtr &f){
	if(picking){
		ppFeedback.progress = (float) f->progress;
		ppServer.publishFeedback(ppFeedback);
	}else if(releasing){
		rpFeedback.progress = (float) f->progress;
		rpServer.publishFeedback(rpFeedback);
	}else{
		dtFeedback.progress = (float) f->progress;
		dtServer.publishFeedback(dtFeedback);
	}
}

void Control::driveToFinish(const actionlib::SimpleClientGoalState& state,
							const robot_control::navDriveToResultConstPtr& r){
	navDriveToFinished = true;
	success = r->success;

}

void Control::driveTo(const robot_control::driveToGoalConstPtr &goal){
	ROS_INFO("%sControl: Recebeu driveTo", KCYN);
	goalx = goal->posx;
	goaly = goal->posy;
	picking = false;
	releasing = false;
	driveToAction();

	dtResult.success = success;

	if(dtResult.success)
		dtServer.setSucceeded(dtResult);
	else
		dtServer.setPreempted(dtResult);
}


void Control::pickProduct(const robot_control::pickProductGoalConstPtr &goal){

	//ROS_INFO("%sControl: Recebeu pickProduct", KCYN);
	gripperReleaseCall();
	goalx = goal->posx;
	goaly = goal->posy;
	picking = true;
	releasing = false;
	ROS_INFO("%sControl: Calling driveToAction", KCYN);
	driveToAction();

	//Call close gripper hand
	if(success)
		gripperGraspCall();

	ROS_INFO("%sControl: driveToAction finished", KCYN);
	ppResult.success = success;

	if(ppResult.success)
		ppServer.setSucceeded(ppResult);
	else
		ppServer.setPreempted(ppResult);
}

void Control::releaseProduct(const robot_control::releaseProductGoalConstPtr &goal){

	//ROS_INFO("%sControl: Recebeu releaseProduct", KCYN);
	goalx = goal->posx;
	goaly = goal->posy;
	picking = false;
	releasing = true;
	driveToAction();
	//Open Gripper
	gripperReleaseCall();

	rpResult.success = success;

	if(rpResult.success)
		rpServer.setSucceeded(rpResult);
	else
		rpServer.setPreempted(rpResult);
}

void Control::gripperGraspCall(){
	if(!graspObjectClient)
		graspObjectClient =  node.serviceClient<robot_control::productManipulationGrasp>(PRODUCTMANIPULATIONGRASP);

	graspObjectClient.call(graspObjSrv);
	bool success = graspObjSrv.response.success;
}

void Control::gripperReleaseCall(){
	if(!releaseObjectClient)
		releaseObjectClient =  node.serviceClient<robot_control::productManipulationRelease>(PRODUCTMANIPULATIONRELEASE);

	releaseObjectClient.call(releaseObjSrv);
	bool success = releaseObjSrv.response.success;
}


void Control::defineGlobalPathCall(){

	//TODO: Call gridMapping to cast positions to map positions
	//TODO: Find out why sometimes the GPS driver from gazebo does not work and the localization do not update with the position
	ros::Rate r(2);
	while(posx == 0.0f || posy == 0.0f){
		ROS_INFO("Waiting for robot's position...");
		r.sleep();
	}

	pathSrv.request.map = map;
	pathSrv.request.beginx = floor(posx * 10);
	pathSrv.request.beginy = floor(posy * 10);
	pathSrv.request.endx = floor(goalx * 10);
	pathSrv.request.endy = floor(goaly * 10);

	//ROS_INFO("%sCONTROL: definePath from  (%.2f, %.2f) to (%.2f, %.2f)", KCYN, posx, posy, goalx, goaly);

	if(!defineGlobalPathClient)
		defineGlobalPathClient = node.serviceClient<robot_control::pathPlanningDefineGlobalPath>(PATHPLANNINGDEFINEPATH);

	defineGlobalPathClient.call(pathSrv);
	xpath = pathSrv.response.xpath;
	ypath = pathSrv.response.ypath;
	//ROS_INFO("%sControl: XPATH size = %d, YPATH size = %d", KCYN, (int) xpath.size(), (int) ypath.size());
}

/*
*
*
*	Map Environment
*
*
*/

void Control::mapEnvironment(const robot_control::mapEnvironmentGoalConstPtr &goal){

	/*
	*
	*	Loading Map from file
	*	Used as shortcut while developing
	* 	{
	 */
	/*
	std::streampos begin,end;
	std::string line;
  	std::ifstream mapfile ("../../ROSWS/map.txt", std::ios::in);
  	begin = mapfile.tellg();
  	mapfile.seekg(0, std::ios::end);
  	end = mapfile.tellg();
  	int i;
  	if(end - begin > 100){
	  	mapfile.seekg(0, std::ios::beg);
	  	getline(mapfile, line);
  		if((int)(line[0] - '0')){
  			ROS_INFO("%sControl: Reading from map file", KCYN);
			if(!creatingMap){
		  		while(getline(mapfile, line)){
		  			for(i = 0; i < line.size(); i++)
		  				map.push_back((float)(line[i] - '0'));
		  		}
				creatingMap = true;
		  	}
		  	meResult.success = true;
			callUpdateMapInformation();
		  	meServer.setSucceeded(meResult);
		  	return;
	  	}else
  			ROS_INFO("%sControl: map file starts with 0!", KCYN);
  	}else
  		ROS_INFO("%sControl: invalid map file!", KCYN);
  	mapfile.close();
  	*/
  	/*
  	*	}
  	*	Loading Map from File End
  	 */
	

	active = true;
	success = true;

	//Wait for navigation action server to connect
	fwClient.waitForServer();
	
	fwGoal.rightWall = (bool) goal->rightWall;
	fwGoal.posx = (float) goal->posx;
	fwGoal.posy = (float) goal->posy;

	fwClient.sendGoal(fwGoal,
		boost::bind(&Control::followWallFinish, this, _1, _2),
		followWallClient::SimpleActiveCallback(),
		boost::bind(&Control::followWallUpdateHandler, this, _1));

	mappingCall();

	while(active){
		//Check for a preempt from the client
		//ROS Wiki - 1.2.1 line 48-55 
		//(http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29)
		if(meServer.isPreemptRequested() || !ros::ok()){
			meServer.setPreempted();
			success = false;
			break;
		}
	}

	meResult.success = success;

	//Call for a update in the Map Information map
	callUpdateMapInformation();

	if(meResult.success)
		meServer.setSucceeded(meResult);
	else
		meServer.setPreempted(meResult);
	
}

void Control::followWallUpdateHandler(const robot_control::followWallFeedbackConstPtr &f){
	meFeedback.progress = (float) f->progress;
	mappingCall();
	meServer.publishFeedback(meFeedback);
}

void Control::followWallFinish(
	const actionlib::SimpleClientGoalState& state,
	const robot_control::followWallResultConstPtr& r){

	success = r->success;
	active = false;
	//ROS_INFO("%sControl: Navigation Finished!", KCYN);

}

void Control::callUpdateMapInformation(){

	updateMapSrv.request.map = map;

	if(!updateMapClient)
		updateMapClient = node.serviceClient<robot_control::mapInformationUpdateGridMap>(MAPINFOUPDATEGRID);

	updateMapClient.call(updateMapSrv);
	bool success = updateMapSrv.response.success;
}

void Control::callGetMapInformation(){

	if(!getMapClient)
		getMapClient = node.serviceClient<robot_control::mapInformationGetGridMap>(MAPINFOGETGRID);
	getMapClient.call(getMapSrv);

	map = getMapSrv.response.map;
}

void Control::mappingCall(){
	
	mappingSrv.request.posx = posx;
	mappingSrv.request.posy = posy;
	mappingSrv.request.posz = posz;
	mappingSrv.request.orientation = orientation;
	mappingSrv.request.cellsize = cellsize;
	mappingSrv.request.width = mapwidth;
	mappingSrv.request.height = mapheight;

	if(!createGridMapClient)
		createGridMapClient = node.serviceClient<robot_control::gridMappingCreateMap>(MAPPINGCREATEGRID);

	createGridMapClient.call(mappingSrv);

	map = mappingSrv.response.map;
	mapsize = mappingSrv.response.size;
}

/*
*
*
*	Other Servers
*
*
*/

void Control::localizationHandler(const robot_control::localization::ConstPtr& localization){

	posx = localization->posx;
	posy = localization->posy;
	posz = localization->posz;
	orientation = localization->orientation;

}

void Control::navStopCall(){
	navStopSrv.request.priority = 1;
	if(!navigationStopClient)
		navigationStopClient = node.serviceClient<robot_control::deliberativeNavigationStop>(NAVSTOPSERVICE);

	navigationStopClient.call(navStopSrv);

}

//TODO: Implement this togheter with transport agent's similar methods
bool Control::stop(robot_control::controlStop::Request &req,
			  	   robot_control::controlStop::Response &res){
	return true;
}

bool Control::reportState(robot_control::controlReportState::Request &req,
						  robot_control::controlReportState::Response &res){
	return true;
}

//Calculate the distance needed to travel between two points
bool Control::returnPathDistance(robot_control::controlReturnPathDistance::Request &req,
						robot_control::controlReturnPathDistance::Response &res){

	int i;
	float totalDistance = 0.0f;

	//Get any updates on current map
	callGetMapInformation();

	goalx = req.goalx;
	goaly = req.goaly;
	defineGlobalPathCall();
	
	//Iterates the points inside the path and sum its distance to the next until the last one
	for(i = 1; i < (int)xpath.size(); i++)
		totalDistance += sqrt(pow(xpath[i] - xpath[i-1], 2) + pow(ypath[i] - ypath[i-1], 2));

	res.distance = totalDistance;
}

/*
*
*
*	Main
*
*
*/

int main(int argc, char **argv){

	ros::init(argc, argv, "control");

	ros::NodeHandle node;

	Control *controller = new Control(node);

	ROS_INFO("Control Running!!!");

	ros::spin();
}