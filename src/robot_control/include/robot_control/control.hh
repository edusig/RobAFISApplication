#include "robot_control/base.h"
#include <iostream>
#include <fstream>
#include <string>

//Client Services
#include "robot_control/deliberativeNavigationStatus.h"
#include "robot_control/deliberativeNavigationStop.h"
#include "robot_control/gridMappingCreateMap.h"
#include "robot_control/pathPlanningDefineGlobalPath.h"
#include "robot_control/mapInformationUpdateGridMap.h"
#include "robot_control/mapInformationGetGridMap.h"
#include "robot_control/productManipulationGrasp.h"
#include "robot_control/productManipulationRelease.h"

//Server Services
#include "robot_control/controlStop.h"
#include "robot_control/controlReportState.h"
#include "robot_control/controlReturnPathDistance.h"

//Client Actions
#include "robot_control/followWallAction.h"
#include "robot_control/navDriveToAction.h"
#include "robot_control/dockingAction.h"

//Server Actions
#include "robot_control/mapEnvironmentAction.h"
#include "robot_control/driveToAction.h"
#include "robot_control/pickProductAction.h"
#include "robot_control/releaseProductAction.h"

//Topics
#include "robot_control/localization.h"

//Helper typedefs for Action client and servers
typedef actionlib::SimpleActionClient<robot_control::followWallAction> followWallClient;
typedef actionlib::SimpleActionClient<robot_control::navDriveToAction> navDriveToClient;
typedef actionlib::SimpleActionClient<robot_control::navDriveToAction> navDockingClient;

typedef actionlib::SimpleActionServer<robot_control::mapEnvironmentAction> mapEnvironmentServer;
typedef actionlib::SimpleActionServer<robot_control::driveToAction> driveToServer;
typedef actionlib::SimpleActionServer<robot_control::pickProductAction> pickProductServer;
typedef actionlib::SimpleActionServer<robot_control::releaseProductAction> releaseProductServer;

class Control{
public:
	Control(ros::NodeHandle);
	virtual ~Control();

	//Drive To
	void driveTo(const robot_control::driveToGoalConstPtr &goal);
	void driveToUpdateHandler(const robot_control::navDriveToFeedbackConstPtr &f);
	void driveToFinish(const actionlib::SimpleClientGoalState& state,
						  const robot_control::navDriveToResultConstPtr& r);
	void driveToAction();
	//Pick Product
	void pickProduct(const robot_control::pickProductGoalConstPtr &goal);
	//Release Product
	void releaseProduct(const robot_control::releaseProductGoalConstPtr &goal);

	//Map Environment
	void mapEnvironment(const robot_control::mapEnvironmentGoalConstPtr &goal);
	void followWallUpdateHandler(const robot_control::followWallFeedbackConstPtr &f);
	void followWallFinish(const actionlib::SimpleClientGoalState& state,
						  const robot_control::followWallResultConstPtr& r);
	void callUpdateMapInformation();
	void callGetMapInformation();

	//Other Services
	void localizationHandler(const robot_control::localization::ConstPtr& localization);
	bool stop(robot_control::controlStop::Request &req,
			  robot_control::controlStop::Response &res);
	bool reportState(robot_control::controlReportState::Request &req,
					 robot_control::controlReportState::Response &res);
	bool returnPathDistance(robot_control::controlReturnPathDistance::Request &req,
							robot_control::controlReturnPathDistance::Response &res);

private:
	enum controlStates{RESTING, DELIVERING, COLLECTING, MAPPING, WAITING};
	ros::NodeHandle node;
	bool active, success, pathDefined;
	bool navDriveToFinished;
	bool creatingMap;
	bool picking, releasing, backwards;
	float posx, posy, posz, orientation;
	float goalx, goaly;
	float mapwidth, mapheight, cellsize;
	int mapsize;
	std::vector<float> map;
	std::vector<float> xpath, ypath;

	//Topic Subscribers
	ros::Subscriber localization;

	//Service Client
	ros::ServiceClient navigationStopClient;
	ros::ServiceClient createGridMapClient;
	ros::ServiceClient defineGlobalPathClient;
	ros::ServiceClient updateMapClient;
	ros::ServiceClient getMapClient;
	ros::ServiceClient graspObjectClient;
	ros::ServiceClient releaseObjectClient;
	//Serivce Client Structs
	robot_control::deliberativeNavigationStop navStopSrv;
	robot_control::gridMappingCreateMap mappingSrv;
	robot_control::pathPlanningDefineGlobalPath pathSrv;
	robot_control::mapInformationUpdateGridMap updateMapSrv;
	robot_control::mapInformationGetGridMap getMapSrv;
	robot_control::productManipulationGrasp graspObjSrv;
	robot_control::productManipulationRelease releaseObjSrv;

	//Service Server
	ros::ServiceServer advertiseStop;
	ros::ServiceServer advertiseReportState;
	ros::ServiceServer advertiseReturnPathDistance;
	//Service Server Structs
	robot_control::controlStop stopService;
	robot_control::controlReportState reportStateService;
	robot_control::controlReturnPathDistance returnPathDistanceService;

	//Action Server progress and result
	robot_control::mapEnvironmentFeedback meFeedback;
	robot_control::mapEnvironmentResult meResult;
	robot_control::driveToFeedback dtFeedback;
	robot_control::driveToResult dtResult;
	robot_control::pickProductFeedback ppFeedback;
	robot_control::pickProductResult ppResult;
	robot_control::releaseProductFeedback rpFeedback;
	robot_control::releaseProductResult rpResult;

	//Action Client Goals
	robot_control::followWallGoal fwGoal;
	robot_control::navDriveToGoal ndtGoal;
	robot_control::dockingGoal ndGoal;

	//Action Clients
	followWallClient fwClient;
	navDriveToClient ndtClient;
	navDockingClient ndClient;

	//Action Server
	mapEnvironmentServer meServer;
	driveToServer dtServer;
	pickProductServer ppServer;
	releaseProductServer rpServer;

	//Service Call Helpers
	void mappingCall();
	void navStopCall();
	void defineGlobalPathCall();
	void gripperGraspCall();
	void gripperReleaseCall();
};