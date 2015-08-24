#include "robot_control/base.h"

//Server Services
#include "robot_control/robAFISApplicationTransportProduct.h"
//Client Services
#include "robot_control/transportAgentReturnPathDistance.h"

//Client Actions
#include "robot_control/mapEnvironmentAction.h"
#include "robot_control/collectProductAction.h"
#include "robot_control/deliverProductAction.h"
#include "robot_control/returnToRestAction.h"

//Server Actions
#include "robot_control/createMapAction.h"

//Clients Actions 
typedef actionlib::SimpleActionClient<robot_control::collectProductAction> collectProductClient;
typedef actionlib::SimpleActionClient<robot_control::deliverProductAction> deliverProductClient;
typedef actionlib::SimpleActionClient<robot_control::returnToRestAction> returnToRestClient;
typedef actionlib::SimpleActionClient<robot_control::mapEnvironmentAction> mapEnvironmentClient;

//Server Actions
typedef actionlib::SimpleActionServer<robot_control::createMapAction> createMapServer;

//Transport Agents states
enum robotPickupStates{ STANDBY, COLLECTING, DELIVERING, RETURNINGTOREST };

//Simple 2D point struct
typedef struct{
	float x;
	float y;
} Point;

//Represents robot's caracteristics and flags
struct Robot{
	Robot( std::string _name, int _ra ) : state(STANDBY), name(_name), restArea(_ra), taskFinished(false){};
	Robot() : state(STANDBY), name("Robot"){};

	//Task Variables
	robotPickupStates state;
	bool taskFinished; //Used to change states
	bool product1; //Which product is it working with
	bool stateChanged; //Flag for calling next action when state changed
	int restArea; //Which rest area is it occuping

	//Mapping Variables
	bool finishedMapping; //Flag for changing states when mapping
	bool stageChanged; //Flag for calling next action when mapping stage changed
	float progress; // Mapping or Task progress
	float oldProgress; //Mapping or Task progress to calc diff

	std::string name; //Identification when debugging
};

class Application{
public:
	Application(ros::NodeHandle);
	~Application();

	//Transport Product Service Server
	bool transportProduct(robot_control::robAFISApplicationTransportProduct::Request &req, 
						  robot_control::robAFISApplicationTransportProduct::Response &res);
	//Start transporting loop
	void startTransporting();
	//Call Transport Agent apropriate service/action for transporting the product
	void callTransportProduct(bool);

	//Action Client (Related to Transport Agent) Update Handler and Finish
	void collectProductUpdateHandler(const robot_control::collectProductFeedbackConstPtr& f);
	void collectProductFinish(const actionlib::SimpleClientGoalState& state,
							  const robot_control::collectProductResultConstPtr& r);
	void deliverProductUpdateHandler(const robot_control::deliverProductFeedbackConstPtr& f);
	void deliverProductFinish(const actionlib::SimpleClientGoalState& state,
							  const robot_control::deliverProductResultConstPtr& r);
	void returnToRestUpdateHandler(const robot_control::returnToRestFeedbackConstPtr& f);
	void returnToRestFinish(const actionlib::SimpleClientGoalState& state,
							  const robot_control::returnToRestResultConstPtr& r);

	//A way of grouping callback from the Update Handler and Finish methods above
	void taskProgress(bool robot1, float progress);
	void taskFinished(bool robot1, bool success);


	//Call Transport Agent create map action
	void createMap(const robot_control::createMapGoalConstPtr &goal);
	void createMapUpdateHandler(const robot_control::mapEnvironmentFeedbackConstPtr& f);
	void createMapFinish(const actionlib::SimpleClientGoalState& state,
						 const robot_control::mapEnvironmentResultConstPtr& r);

	//Returns a pointer to one robot struct
	//Since there was only two robots, if bool robot1 is true it returns a pointer to r1 or else it returns a pointer to r2
	Robot* getRobotPointer(bool robot1);
	
	//Main loop
	void update();

private:
	ros::NodeHandle node;
	//Default is 2 stages, both robots go to the other side of the map(stage1) then comeback (stage2)
	int mappingStage;
	//Flag for stopping action loops
	bool active;
	//Flag for stopping transporting loop
	bool transporting;
	//Simple time keeping
	unsigned int time;
	//Position for the goal
	float goalx, goaly;
	//Holds products when both robots are busy (FILO)
	std::vector<Point> productQueue;
	//Collection of points to pass for Trasnport Agent create map service
	std::vector<Point> mappingPoints;
	//Matrix of rest, storage and product areas
	std::vector< std::vector<Point> > predefinedAreas;
	//Not ocupied rest areas
	std::vector<bool> restAreasAvailable;

	Robot r1, r2;

	//Service Server Provider (Advertiser)
	ros::ServiceServer advertiseTransportProduct;

	//Service Client
	ros::ServiceClient returnPathDistance1, returnPathDistance2;
	robot_control::transportAgentReturnPathDistance pathDistanceSrv;

	//Action Server Variables
	robot_control::createMapFeedback cmFeedback;
	robot_control::createMapResult cmResult;
	robot_control::collectProductFeedback cpFeedback;
	robot_control::collectProductResult cpResult;
	robot_control::deliverProductFeedback dpFeedback;
	robot_control::deliverProductResult dpResult;
	robot_control::returnToRestFeedback rtFeedback;
	robot_control::returnToRestResult rtResult;


	//Action Client Goals
	robot_control::mapEnvironmentGoal meGoal;
	robot_control::collectProductGoal cpGoal;
	robot_control::deliverProductGoal dpGoal;
	robot_control::returnToRestGoal rtGoal;
	
	//Action Server and Client objects
	createMapServer meServer;
	mapEnvironmentClient meClient1, meClient2;
	collectProductClient cpClient1, cpClient2;
	deliverProductClient dpClient1, dpClient2;
	returnToRestClient rtClient1, rtClient2;

	void initializePredefinedAreas();
	bool comparePathLength(Point);

	//Call a TransportAgent create map action
	void callCreateMap(bool, mapEnvironmentClient*);
};