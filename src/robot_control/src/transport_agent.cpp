#include "robot_control/transport_agent.hh"

TransportAgent::TransportAgent(ros::NodeHandle n):
	//Start action servers and connect to action clients
	//Must be set here so it can be used within the class
	meClient(CONTROLMAPENVIRONMENT, true),
	dtClient(CONTROLDRIVETO, true),
	ppClient(CONTROLPICKPRODUCT, true),
	rpClient(CONTROLRELEASEPRODUCT, true),
	meServer(n, TRANSPORTAGENTMAPENVIRONMENT, boost::bind(&TransportAgent::mapEnvironment, this, _1), false),
	cpServer(n, TRANSPORTAGENTCOLLECTPRODUCT, boost::bind(&TransportAgent::collectProduct, this, _1), false),
	dpServer(n, TRANSPORTAGENTDELIVERPRODUCT, boost::bind(&TransportAgent::deliverProduct, this, _1), false),
	rtServer(n, TRANSPORTAGENTRETURNINGTOREST, boost::bind(&TransportAgent::returnToRest, this, _1), false)
{

	node = n;
	active = true;

	//Path Distance Service
	returnPathDistanceClient = node.serviceClient<robot_control::controlReturnPathDistance>(CONTROLRETURNPATHDISTANCE);

	//Advertise Services
	advertiseAbort = node.advertiseService(TRANSPORTAGENTABORT, &TransportAgent::abort, this);
	advertiseReport = node.advertiseService(TRANSPORTAGENTREPORT, &TransportAgent::report, this);
	advertiseReturnPathDistance = node.advertiseService(TRANSPORTAGENTRETURNPATHDISTANCE, &TransportAgent::returnPathDistance, this);

	//Start Map Environment Action Server
	meServer.start();
	//Start Collect Product Action Server
	cpServer.start();
	//Start Deliver Product Action Server
	dpServer.start();
	//Start Return To Rest Action Server
	rtServer.start();
}

TransportAgent::~TransportAgent(){}

/*
*
*
*	Map Environment Methods
*
*
*/

void TransportAgent::mapEnvironment(
	const robot_control::mapEnvironmentGoalConstPtr &goal){

	success = true;
	active = true;

	meClient.waitForServer();
	meClientGoal.posx = (float) goal->posx;
	meClientGoal.posy = (float) goal->posy;
	meClientGoal.rightWall = (bool) goal->rightWall;
	robot1 = goal->robot1;

	meClient.sendGoal(meClientGoal, 
		boost::bind(&TransportAgent::mapEnvironmentFinish, this, _1, _2), //Finish action handler 
		mapEnvironmentClient::SimpleActiveCallback(),  //Activate action handler
		boost::bind(&TransportAgent::mapEnvironmentUpdateHandler, this, _1)); //Update action handler	

	//Action loop
	while(active){

		//Check for a preempt from the client
		//ROS Wiki - 1.2.1 line 48-55 
		//(http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29)
		if(meServer.isPreemptRequested() || !ros::ok()){
			meServer.setPreempted();
			break;
		}
	}

	meResult.success = success;
	meResult.robot1 = robot1;
	
	if(meResult.success)
		meServer.setSucceeded(meResult);
	else
		meServer.setPreempted(meResult);
}

void TransportAgent::mapEnvironmentUpdateHandler(
	const robot_control::mapEnvironmentFeedbackConstPtr& f){
	meFeedback.progress = f->progress;
	meFeedback.robot1 = robot1;
	meServer.publishFeedback(meFeedback);
}

void TransportAgent::mapEnvironmentFinish(const actionlib::SimpleClientGoalState& state,
										  const robot_control::mapEnvironmentResultConstPtr& r){
	success = r->success;
	active = false;
	
}

/*
*
*
*	Collect Product Methods
*
*
*/

void TransportAgent::collectProduct(const robot_control::collectProductGoalConstPtr &goal){

	float goalx = goal->posx;
	float goaly = goal->posy;
	bool robot1 = goal->robot1;
	cpFeedback.robot1 = robot1;
	cpResult.robot1 = robot1;

	success = true;
	active = true;

	ppClient.waitForServer();
	ppClientGoal.posx = goalx;
	ppClientGoal.posy = goaly;

	ppClient.sendGoal(ppClientGoal, 
		boost::bind(&TransportAgent::pickProductFinish, this, _1, _2), 
		pickProductClient::SimpleActiveCallback(), 
		boost::bind(&TransportAgent::pickProductUpdateHandler, this, _1));
	ROS_INFO("%sTA: Enviou pickProduct para Control", KGRN);
	while(active){

		//Updates progress
		if(progress != cpFeedback.progress){
			cpFeedback.progress = progress;
			cpServer.publishFeedback(cpFeedback);
		}

		//Check for a preempt from the client
		//ROS Wiki - 1.2.1 line 48-55 
		//(http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29)
		if(cpServer.isPreemptRequested() || !ros::ok()){
			cpServer.setPreempted();
			success = false;
			break;
		}
	}

	cpResult.success = success;
	if(cpResult.success)
		cpServer.setSucceeded(cpResult);
	else
		cpServer.setPreempted(cpResult);

}

/*
*
*
*	Deliver Product
*
*
 */

void TransportAgent::deliverProduct(const robot_control::deliverProductGoalConstPtr &goal){

	float goalx = goal->posx;
	float goaly = goal->posy;
	bool robot1 = goal->robot1;
	dpFeedback.robot1 = robot1;
	dpResult.robot1 = robot1;

	success = true;
	active = true;

	ROS_INFO("%sTA: Recebeu deliver product action", KGRN);
	rpClient.waitForServer();
	rpClientGoal.posx = goalx;
	rpClientGoal.posy = goaly;

	rpClient.sendGoal(rpClientGoal, 
		boost::bind(&TransportAgent::releaseProductFinish, this, _1, _2), 
		releaseProductClient::SimpleActiveCallback(), 
		boost::bind(&TransportAgent::releaseProductUpdateHandler, this, _1));
	ROS_INFO("%sTA: Enviou releaseProduct para Control", KGRN);
	while(active){

		//Updates feedback
		if(progress != dpFeedback.progress){
			dpFeedback.progress = progress;
			dpServer.publishFeedback(dpFeedback);
		}

		//Check for a preempt from the client
		//ROS Wiki - 1.2.1 line 48-55 
		//(http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29)
		if(dpServer.isPreemptRequested() || !ros::ok()){
			dpServer.setPreempted();
			success = false;
			break;
		}

	}

	dpResult.success = success;
	if(dpResult.success)
		dpServer.setSucceeded(dpResult);
	else
		dpServer.setPreempted(dpResult);

}

/*
*
*
*	Return To Rest
*
*
 */

void TransportAgent::returnToRest(const robot_control::returnToRestGoalConstPtr &goal){

	float goalx = goal->posx;
	float goaly = goal->posy;
	bool robot1 = goal->robot1;
	rtFeedback.robot1 = robot1;
	rtResult.robot1 = robot1;

	success = true;
	active = true;

	ROS_INFO("%sTA: Recebeu return to rest action", KGRN);
	dtClient.waitForServer();
	dtClientGoal.posx = goalx;
	dtClientGoal.posy = goaly;

	dtClient.sendGoal(dtClientGoal, 
		boost::bind(&TransportAgent::driveToFinish, this, _1, _2), 
		driveToClient::SimpleActiveCallback(), 
		boost::bind(&TransportAgent::driveToUpdateHandler, this, _1));
	ROS_INFO("%sTA: Enviou driveTo para Control", KGRN);
	while(active){

		//Updates feedback
		if(progress != dpFeedback.progress){
			rtFeedback.progress = progress;
			rtServer.publishFeedback(rtFeedback);
		}

		//Check for a preempt from the client
		//ROS Wiki - 1.2.1 line 48-55 
		//(http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29)
		if(rtServer.isPreemptRequested() || !ros::ok()){
			rtServer.setPreempted();
			success = false;
			break;
		}
	}

	rtResult.success = success;
	if(rtResult.success)
		rtServer.setSucceeded(rtResult);
	else
		rtServer.setPreempted(rtResult);

}

/*
*
*
*	Drive To Action Handlers
*
*
 */

void TransportAgent::driveToUpdateHandler(const robot_control::driveToFeedbackConstPtr& f){
	progress = f->progress;
}

void TransportAgent::driveToFinish(const actionlib::SimpleClientGoalState& state,
								   const robot_control::driveToResultConstPtr& r){
	success = (bool) r->success;
	active = false;
}

/*
*
*
*	Pick Product Action Handlers
*
*
 */
void TransportAgent::pickProductUpdateHandler(const robot_control::pickProductFeedbackConstPtr& f){
	progress = f->progress;
}

void TransportAgent::pickProductFinish(const actionlib::SimpleClientGoalState& state,
								   		  const robot_control::pickProductResultConstPtr& r){
	success = (bool) r->success;
	active = false;
}

/*
*
*
*	Release Product Action Handler
*
*
 */
void TransportAgent::releaseProductUpdateHandler(const robot_control::releaseProductFeedbackConstPtr& f){
	progress = f->progress;
}

void TransportAgent::releaseProductFinish(const actionlib::SimpleClientGoalState& state,
								   		  const robot_control::releaseProductResultConstPtr& r){
	success = (bool) r->success;
	active = false;
}

/*
*
*
*	Other Service Methods
*
*
 */

bool TransportAgent::abort(robot_control::transportAgentAbort::Request &req,
						   robot_control::transportAgentAbort::Response &res){
	res.success = true;
	return true;
}

bool TransportAgent::report(robot_control::transportAgentReport::Request &req,
							robot_control::transportAgentReport::Response &res){
	res.success = true;
	return true;
}

//Calculate the distance needed to travel between two points
bool TransportAgent::returnPathDistance(
	robot_control::transportAgentReturnPathDistance::Request &req,
	robot_control::transportAgentReturnPathDistance::Response &res){

	pathDistanceSrv.request.goalx = req.goalx;
	pathDistanceSrv.request.goaly = req.goaly;

	//If server have not been initialized or disconnected, initialize/reconnect it.
	if(!returnPathDistanceClient)
		returnPathDistanceClient = node.serviceClient<robot_control::controlReturnPathDistance>(CONTROLRETURNPATHDISTANCE);


	//ROS_INFO("%sTA: goalx = %.2f | goaly = %.2f", KGRN, req.goalx, req.goaly);
	returnPathDistanceClient.call(pathDistanceSrv);
	res.distance = pathDistanceSrv.response.distance;
	//ROS_INFO("%sTA: Path Distance = %.2f", KGRN, res.distance);

	return true;
}

/*
*
*
*	Main
*
*
 */

int main(int argc, char **argv){

	ros::init(argc, argv, "transportAgent");

	ros::NodeHandle node;

	TransportAgent *transportAgent = new TransportAgent(node);

	ROS_INFO("TransportAgent Running!!!");

	ros::spin();

}
