#include "robot_control/base.h"

//Client Actions
#include "robot_control/createMapAction.h"
//Client Service
#include "robot_control/robAFISApplicationTransportProduct.h"
#include "robot_control/robAFISApplicationCreateMap.h"

typedef actionlib::SimpleActionClient<robot_control::createMapAction> createMapClient;

class Operation{
public:
	Operation(ros::NodeHandle);
	virtual ~Operation();

	void createMap();
	void createMapUpdateHandler(const robot_control::createMapFeedbackConstPtr&);
	void createMapFinish(const actionlib::SimpleClientGoalState& state,
						 const robot_control::createMapResultConstPtr& result);
	void addProductRequest(bool product1);
	void userInterface();
	void update();

private:
	//Menu option
	int op;
	//Loop control
	bool active;
	//Flag for mapping
	bool mapped;

	createMapClient client;

	ros::NodeHandle node;
	ros::ServiceClient transportProductClient;

	robot_control::createMapGoal goal;

	//Service Client object
	robot_control::robAFISApplicationTransportProduct transportProductSrv;
	//Configure rest, product and storage areas position
	void initializaPredefinedAreas();
};