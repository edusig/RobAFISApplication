#include "robot_control/application.hh"

Application::Application(ros::NodeHandle n): 
	//Subscribe to services
	meClient1(R1TRANSPORTAGENTMAPENVIRONMENT, true),
	cpClient1(R1TRANSPORTAGENTCOLLECTPRODUCT, true),
	dpClient1(R1TRANSPORTAGENTDELIVERPRODUCT, true),
	rtClient1(R1TRANSPORTAGENTRETURNINGTOREST, true),
	meClient2(R2TRANSPORTAGENTMAPENVIRONMENT, true),
	cpClient2(R2TRANSPORTAGENTCOLLECTPRODUCT, true),
	dpClient2(R2TRANSPORTAGENTDELIVERPRODUCT, true),
	rtClient2(R2TRANSPORTAGENTRETURNINGTOREST, true),
	meServer(n, APPLICATIONCREATEMAP, boost::bind(&Application::createMap, this, _1), false){

	node = n;

	initializePredefinedAreas();

	//Initialize robots' information
	r1 = Robot(std::string("Robot1"), 0);
	r2 = Robot(std::string("Robot2"), 0);

	//Initialize stop points for environmental mapping
	Point p0, p1;
	p0.x = 9.5f;
	p0.y = 0.5f;
	p1.x = 0.5f;
	p1.y = 9.5f;
	mappingPoints.push_back(p0);
	mappingPoints.push_back(p1);

	//Transport Service advertise
	advertiseTransportProduct = node.advertiseService(APPLICATIONTRANSPORTPRODUCT, &Application::transportProduct, this);

	//Path Distance Service connect
	returnPathDistance1 = node.serviceClient<robot_control::transportAgentReturnPathDistance>(R1TRANSPORTAGENTRETURNPATHDISTANCE); //ROBOT 1 Distance
	returnPathDistance2 = node.serviceClient<robot_control::transportAgentReturnPathDistance>(R2TRANSPORTAGENTRETURNPATHDISTANCE); //ROBOT 2 Distance
	
	//Map Environment Action
	meServer.start();
}

Application::~Application(){}

/*
*
*
*	Transport Product Service Methods
*
*
 */

//For reference check file srv/robAFISApplicationTransportProduct.srv
//Receives a transport order and adds it to the queue
//Activates transporting loop
bool Application::transportProduct(
	robot_control::robAFISApplicationTransportProduct::Request &req,
	robot_control::robAFISApplicationTransportProduct::Response &res){
	
	//Create a point, with the requested position x and y, and put it on the transport Queue
	Point p;
	p.x = predefinedAreas[1][req.product1?0:1].x;
	p.y = predefinedAreas[1][req.product1?0:1].y;
	productQueue.push_back(p);
	
	//Used in the main loop
	transporting = true;
	return true;

}

//Transporting loop
void Application::startTransporting(){

	Point goal;
	//If there are no products and both robots are resting stop transporting loop
	if(productQueue.size() == 0 && r1.state == STANDBY && r2.state == STANDBY){
		//Stop transporting (in the main loop)
		transporting = false;
		return;
	}

	if(productQueue.size() > 0){

		//If both robots are resting choose the one with the nearest path to go pick up
		if(r1.state == STANDBY && r2.state == STANDBY){
			goal = productQueue.front();
			callTransportProduct(comparePathLength(goal));
		//If only 1 robot is avaiable call transport product based on the availability of the first
		}else if(r1.state == STANDBY || r2.state == STANDBY){
			callTransportProduct(r1.state == STANDBY);
		//TODO: Change timer to use ros::Time::now()
		}else if(time % 100 == 0)//Every 5 seconds (100/20 = 5)
			ROS_INFO("%sWaiting for available robots. %d products on queue.", KYEL, (int) productQueue.size()); 
	}

	//If a agent is not resting (not on standby) update its progress
	if(r1.state != STANDBY || r2.state != STANDBY){
		//Verify through topics if it finished its last task (collecting, delivering, ...)
		if(r1.taskFinished || r2.taskFinished)
			//If finished, call for the next one
			callTransportProduct(r1.taskFinished);
	}

	//Timer helper for showing message when both robots are occupied
	time++;
}

void Application::callTransportProduct(bool robot1){

	Point goal;
	Robot *robot = getRobotPointer(robot1);
	robot->taskFinished = false;

	//Selecting action clients from robot1 or robot2
	collectProductClient *cpClient = robot1 ? &cpClient1 : &cpClient2;
	deliverProductClient *dpClient = robot1 ? &dpClient1 : &dpClient2;
	returnToRestClient *rtClient = robot1 ? &rtClient1 : &rtClient2;
	ros::ServiceClient *returnPathDistance = robot1 ? &returnPathDistance1 : &returnPathDistance2;

	switch(robot->state){
		case STANDBY: //STANDBY -> COLLECTING
			goal = productQueue.front(); //get first element
			productQueue.erase(productQueue.begin()); //Erease first element

			robot->state = COLLECTING;
			restAreasAvailable[robot->restArea] = true;
			robot->restArea = -1;

			//predefinedAreas[1][0] is product1
			//predefinedAreas[1][1] is product2
			if(goal.x == predefinedAreas[1][0].x && goal.y == predefinedAreas[1][0].y)
				robot->product1 = true;
			else if(goal.x == predefinedAreas[1][1].x && goal.y == predefinedAreas[1][1].y)
				robot->product1 = false;

			//Defines the goal to the client action struct
			cpGoal.posx = goal.x;
			cpGoal.posy = goal.y;
			cpGoal.robot1 = robot1;	

			//Wait for connection to the client
			cpClient->waitForServer();
			cpClient->sendGoal(cpGoal, 
				boost::bind(&Application::collectProductFinish, this, _1, _2), //Finish action Handler
				mapEnvironmentClient::SimpleActiveCallback(), // Activate action handler
				boost::bind(&Application::collectProductUpdateHandler, this, _1)); //Update action handler
			break;
		case COLLECTING: //COLLECTING -> DELIVERING
			goal = robot->product1 ? predefinedAreas[2][0] : predefinedAreas[2][1];
			robot->state = DELIVERING;
			//Defines the goal to the client action struct
			dpGoal.posx = goal.x;
			dpGoal.posy = goal.y;
			dpGoal.robot1 = robot1;	

			//Wait for connection to the client
			dpClient->waitForServer();
			dpClient->sendGoal(dpGoal, 
				boost::bind(&Application::deliverProductFinish, this, _1, _2), //Finish action handler
				mapEnvironmentClient::SimpleActiveCallback(), //Activate action handler
				boost::bind(&Application::deliverProductUpdateHandler, this, _1)); //Update action handler
			break;
		case DELIVERING: //DELIVERING -> RETURNING TO REST
			{
				robot->state = RETURNINGTOREST;
				std::vector<float> rtgoalx;
				std::vector<float> rtgoaly;

				//If service is not defined or connected, define and reconnect
				if(!returnPathDistance)
					*returnPathDistance = node.serviceClient<robot_control::transportAgentReturnPathDistance>(R2TRANSPORTAGENTRETURNPATHDISTANCE);

				//Find the nearest available rest area
				float minDist = 0;
				int i, areaToOccupy;
				//Call pathdistance for each rest area
				for(i = 0; i < (int) predefinedAreas[0].size(); i++){
					if(restAreasAvailable[i]){//if available
						pathDistanceSrv.request.goalx = predefinedAreas[0][i].x;
						pathDistanceSrv.request.goaly = predefinedAreas[0][i].y;
						(*returnPathDistance).call(pathDistanceSrv);
						if(minDist == 0 || pathDistanceSrv.response.distance < minDist){
							rtGoal.posx = pathDistanceSrv.request.goalx;
							rtGoal.posy = pathDistanceSrv.request.goaly;
							areaToOccupy = i; 
							minDist = pathDistanceSrv.response.distance;
						}
					}
				}

				rtGoal.robot1 = robot1;	
				restAreasAvailable[areaToOccupy] = false;
				robot->restArea = areaToOccupy;

				//Wait for connection to the client
				rtClient->waitForServer();
				rtClient->sendGoal(rtGoal, 
					boost::bind(&Application::returnToRestFinish, this, _1, _2), //Finish action handler
					mapEnvironmentClient::SimpleActiveCallback(), //Activate action handler
					boost::bind(&Application::returnToRestUpdateHandler, this, _1)); //Update actinon handler
			}
			break;
		case RETURNINGTOREST: //RETURNING TO REST -> STANDBY
			robot->state = STANDBY;
			robot->oldProgress = -1;
			break;
	}
}

/*
*
*	Collect product Action Handlers
*
*/
void Application::collectProductUpdateHandler(
	const robot_control::collectProductFeedbackConstPtr& f){
	taskProgress(f->robot1, f->progress);
}

void Application::collectProductFinish(const actionlib::SimpleClientGoalState& state,
									   const robot_control::collectProductResultConstPtr& r){
	taskFinished(r->robot1, r->success);
}

/*
*
*	Deliver product Action Handlers
*
*/
void Application::deliverProductUpdateHandler(
	const robot_control::deliverProductFeedbackConstPtr& f){
	taskProgress(f->robot1, f->progress);
}

void Application::deliverProductFinish(const actionlib::SimpleClientGoalState& state,
									   const robot_control::deliverProductResultConstPtr& r){
	taskFinished(r->robot1, r->success);
}

/*
*
*	Return to rest Action Handlers
*
*/
void Application::returnToRestUpdateHandler(
	const robot_control::returnToRestFeedbackConstPtr& f){
	taskProgress(f->robot1, f->progress);
}

void Application::returnToRestFinish(const actionlib::SimpleClientGoalState& state,
									   const robot_control::returnToRestResultConstPtr& r){
	taskFinished(r->robot1, r->success);
}

/*
*
*	Generic Task Handlers
*
*/
void Application::taskProgress(bool robot1, float progress){
	Robot *robot = getRobotPointer(robot1);
	int taskStage = robot->state == DELIVERING ? 1 : 0;
	taskStage = robot->state == RETURNINGTOREST ? 2 : taskStage;
	robot->progress = progress/3 + (taskStage * 100 / 3);
	if(fabs(robot->progress - robot->oldProgress) > 0.1f){
		robot->oldProgress = robot->progress;
		ROS_INFO("%sRobot#%d progress = %.2f", KYEL, ((int)!robot1) + 1, robot->progress);
	}
}

void Application::taskFinished(bool robot1, bool success){
	Robot *robot = getRobotPointer(robot1);
	if(success){
		robot->taskFinished = true;
	}else{
		ROS_INFO("Application: FAILED to complete action %d for %s", robot->state, robot->name.c_str());
	}
}

//Return true if robot1 is closer to the product
bool Application::comparePathLength(Point goal){

	float d1, d2;

	//If service is not set or disconnected, reconnect
	if(!returnPathDistance1)
		returnPathDistance1 = node.serviceClient<robot_control::transportAgentReturnPathDistance>(R1TRANSPORTAGENTRETURNPATHDISTANCE);
	if(!returnPathDistance2)
		returnPathDistance2 = node.serviceClient<robot_control::transportAgentReturnPathDistance>(R2TRANSPORTAGENTRETURNPATHDISTANCE);

	pathDistanceSrv.request.goalx = goal.x;
	pathDistanceSrv.request.goaly = goal.y;

	//Find the distance from both robots to the goal product
	returnPathDistance1.call(pathDistanceSrv);
	d1 = pathDistanceSrv.response.distance;

	returnPathDistance2.call(pathDistanceSrv);
	d2 = pathDistanceSrv.response.distance;

	return d1 < d2;
}

/*
*
*
*	Create Map Service Server Methods
*	
*	
*/

void Application::createMap(const robot_control::createMapGoalConstPtr &goal){
 
	active = true; //Activate action loop
	r1.stateChanged = true;
	r2.stateChanged = true;
	mappingStage = 2;//number of mapping stages

	while(active && mappingStage > 0){

		if(r1.stateChanged)
			callCreateMap(true, &meClient1);

		if(r2.stateChanged)
			callCreateMap(false, &meClient2);

		//Check for a preempt from the client
		//ROS Wiki - 1.2.1 line 48-55 
		//(http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29)
		if(meServer.isPreemptRequested() || !ros::ok()){
			meServer.setPreempted();
			active = false;
			break;
		}

		//When both robots finish mapping, go to the next stage
		if(r1.finishedMapping && r2.finishedMapping){
			
			r1.finishedMapping = false;
			r1.stateChanged = true;

			r2.finishedMapping = false;
			r2.stateChanged = true;
			
			mappingStage--;
		}

		if(mappingStage <= 0){
			cmResult.success = true;
			meServer.setSucceeded(cmResult);
			active = false;
			return;
		}

	}

	if(cmResult.success)
		meServer.setSucceeded(cmResult);
	else
		meServer.setSucceeded(cmResult);
}

//Calls create map action client
void Application::callCreateMap(bool robot1, mapEnvironmentClient *meClient){
	Robot *robot = robot1 ? &r1 : &r2;
	robot->stateChanged = false;


	meClient->waitForServer();

	int index = mappingStage == 2 && !robot1 || mappingStage == 1 && robot1;
	meGoal.posx = mappingPoints[index].x;
	meGoal.posy = mappingPoints[index].y;
	meGoal.rightWall = mappingStage == 2;//Half the times it follows the right wall
	meGoal.robot1 = robot1;
	ROS_INFO("%sApplication: Create map for robot #%d, goal = %.2f, %.2f", KYEL, ((int)!robot1) + 1, meGoal.posx, meGoal.posy);

	meClient->sendGoal(meGoal, 
		boost::bind(&Application::createMapFinish, this, _1, _2), 
		mapEnvironmentClient::SimpleActiveCallback(), 
		boost::bind(&Application::createMapUpdateHandler, this, _1));
}

void Application::createMapUpdateHandler(
	const robot_control::mapEnvironmentFeedbackConstPtr& f){
	
	Robot *robot = f->robot1 ? &r1 : &r2;

	if(meServer.isPreemptRequested() || !ros::ok())
		meServer.setPreempted();

	if(mappingStage <= 0){
		cmResult.success = true;
		meServer.setSucceeded(cmResult);
		active = false;
		return;
	}

	if(fabs(robot->progress - f->progress) > 0.5){
		robot->progress = f->progress;
		cmFeedback.progress = ((r1.progress + r2.progress)/2) / 2 + ((2 - mappingStage) * 50.0 );
		meServer.publishFeedback(cmFeedback);
	}
}

void Application::createMapFinish(const actionlib::SimpleClientGoalState& state,
								  const robot_control::mapEnvironmentResultConstPtr& r){
	
	ROS_INFO("%sApplication: Transport Agent Finished! %d", KYEL, r->robot1);

	Robot *robot = r->robot1 ? &r1 : &r2;
	robot->finishedMapping = true;
	ROS_INFO("%sApplication: R1 FinishedMapping = %d | R2 FinishedMapping = %d", KYEL, r1.finishedMapping, r2.finishedMapping);
	robot->stageChanged = true;
	robot->progress = 0.0f;
	cmResult.success = (bool) r->success;
	
	if(!r->success){
		meServer.setPreempted(cmResult, "Task did not succeeded");
		return;
	}

}

Robot* Application::getRobotPointer(bool robot1){
	return robot1 ? &r1 : &r2;
}

void Application::update(){

	if(transporting)
		startTransporting();

}


void Application::initializePredefinedAreas(){
	Point ra1, ra2, ra3, pa1, pa2, sa1, sa2;
	std::vector<Point> restAreas, productAreas, storageAreas;

	//Rest Area 1 (0.5, 9.5)
	ra1.x = 0.5;
	ra1.y = 9.5;
	//Rest Area 2 (5.0, 5.0)
	ra2.x = 5.0;
	ra2.y = 5.0;
	//Rest Area 3 (9.5, 0.5)
	ra3.x = 9.5;
	ra3.y = 0.5;

	restAreas.push_back(ra1);
	restAreas.push_back(ra2);
	restAreas.push_back(ra3);

	//Product Area 1 (5.0, 2.5)
	pa1.x = 5.0;
	pa1.y = 2.5;
	//Product Area 2 (5.0 7.5)
	pa2.x = 5.0;
	pa2.y = 7.5;

	productAreas.push_back(pa1);
	productAreas.push_back(pa2);

	//Storage Area 1 (0.5, 5.0)
	sa1.x = 0.5;
	sa1.y = 5.0;
	//Storage Area 2 (9.5, 5.0)
	sa2.x = 9.5;
	sa2.y = 5.0;

	storageAreas.push_back(sa1);
	storageAreas.push_back(sa2);

	predefinedAreas.push_back(restAreas);
	predefinedAreas.push_back(productAreas);
	predefinedAreas.push_back(storageAreas);

	restAreasAvailable.resize(3, true);
}
/*
*
*	Main
*
 */

int main(int argc, char **argv){

	ros::init(argc, argv, "application");

	ros::NodeHandle node;

	ros::Rate loop_rate(20);

	Application *application = new Application(node);

	ROS_INFO("Application Running!!!");

	while(ros::ok()){

		application->update();

		ros::spinOnce();
		loop_rate.sleep();

	}
}
