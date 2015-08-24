#include "robot_control/base.h"

//Client Services
#include "robot_control/controlReturnPathDistance.h"

//Server Services
#include "robot_control/transportAgentAbort.h"
#include "robot_control/transportAgentReport.h"
#include "robot_control/transportAgentReturnPathDistance.h"

//Client Actions
#include "robot_control/driveToAction.h"
#include "robot_control/pickProductAction.h"
#include "robot_control/releaseProductAction.h"

//Server Actions
#include "robot_control/mapEnvironmentAction.h"
#include "robot_control/collectProductAction.h"
#include "robot_control/deliverProductAction.h"
#include "robot_control/returnToRestAction.h"

//Helper typedefs for Action client and servers
typedef actionlib::SimpleActionServer<robot_control::mapEnvironmentAction> mapEnvironmentServer;
typedef actionlib::SimpleActionServer<robot_control::collectProductAction> collectProductServer;
typedef actionlib::SimpleActionServer<robot_control::deliverProductAction> deliverProductServer;
typedef actionlib::SimpleActionServer<robot_control::returnToRestAction> returnToRestServer;

typedef actionlib::SimpleActionClient<robot_control::mapEnvironmentAction> mapEnvironmentClient;
typedef actionlib::SimpleActionClient<robot_control::driveToAction> driveToClient;
typedef actionlib::SimpleActionClient<robot_control::pickProductAction> pickProductClient;
typedef actionlib::SimpleActionClient<robot_control::releaseProductAction> releaseProductClient;

class TransportAgent{
public:
	TransportAgent(ros::NodeHandle);
	virtual ~TransportAgent();


	//Map Environment
	void mapEnvironment(const robot_control::mapEnvironmentGoalConstPtr &goal);
	void mapEnvironmentUpdateHandler(const robot_control::mapEnvironmentFeedbackConstPtr& feedback);
	void mapEnvironmentFinish(const actionlib::SimpleClientGoalState& state,
							  const robot_control::mapEnvironmentResultConstPtr& result);

	//Transport Product
	void collectProduct(const robot_control::collectProductGoalConstPtr &goal);
	void pickProductUpdateHandler(const robot_control::pickProductFeedbackConstPtr& f);
	void pickProductFinish(const actionlib::SimpleClientGoalState& state,
						   const robot_control::pickProductResultConstPtr& r);

	void deliverProduct(const robot_control::deliverProductGoalConstPtr &goal);
	void releaseProductUpdateHandler(const robot_control::releaseProductFeedbackConstPtr& f);
	void releaseProductFinish(const actionlib::SimpleClientGoalState& state,
							  const robot_control::releaseProductResultConstPtr& r);

	void returnToRest(const robot_control::returnToRestGoalConstPtr &goal);
	void driveToUpdateHandler(const robot_control::driveToFeedbackConstPtr& f);
	void driveToFinish(const actionlib::SimpleClientGoalState& state,
					   const robot_control::driveToResultConstPtr& r);


	//Other Service Methods
	bool abort(robot_control::transportAgentAbort::Request &req,
			   robot_control::transportAgentAbort::Response &res);
	bool report(robot_control::transportAgentReport::Request &req,
				robot_control::transportAgentReport::Response &res);
	bool returnPathDistance(robot_control::transportAgentReturnPathDistance::Request &req,
							robot_control::transportAgentReturnPathDistance::Response &res);
	bool teleoperate();

private:
	enum agentStates{ RESTING, DELIVERING, COLLECTING, MAPPING, WAITING};
	enum controlState{  };
	bool active, success;
	float posx, posy;
	float progress, oldProgress;
	bool robot1;
	ros::NodeHandle node;

	//Service Servers
	ros::ServiceServer advertiseAbort;
	ros::ServiceServer advertiseReport;
	ros::ServiceServer advertiseReturnPathDistance;

	//Service Server Structs
	robot_control::transportAgentAbort abortService;
	robot_control::transportAgentReport reportService;
	robot_control::transportAgentReturnPathDistance returnPathDistanceService;

	//Service Client
	ros::ServiceClient returnPathDistanceClient;
	//Service Client Struct
	robot_control::controlReturnPathDistance pathDistanceSrv;

	robot_control::mapEnvironmentFeedback meFeedback;
	robot_control::mapEnvironmentResult meResult;
	robot_control::mapEnvironmentGoal meClientGoal;//Server

	robot_control::collectProductFeedback cpFeedback;
	robot_control::collectProductResult cpResult;
	robot_control::collectProductGoal cpClientGoal;

	robot_control::deliverProductFeedback dpFeedback;
	robot_control::deliverProductResult dpResult;
	robot_control::deliverProductGoal dpClientGoal;

	robot_control::returnToRestFeedback rtFeedback;
	robot_control::returnToRestResult rtResult;
	robot_control::returnToRestGoal rtClientGoal;

	robot_control::driveToGoal dtClientGoal;
	robot_control::pickProductGoal ppClientGoal;
	robot_control::releaseProductGoal rpClientGoal;

	//Action Servers
	collectProductServer cpServer;
	deliverProductServer dpServer;
	returnToRestServer rtServer;
	mapEnvironmentServer meServer;

	//Action Clients
	mapEnvironmentClient meClient;
	driveToClient dtClient;
	pickProductClient ppClient;
	releaseProductClient rpClient;

};