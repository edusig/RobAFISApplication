#include "robot_control/base.h"

//Client Services
#include "robot_control/baseDriverSetDrivePower.h"
#include "robot_control/baseDriverSetRotation.h"

//Server Services
#include "robot_control/deliberativeNavigationStatus.h"
#include "robot_control/deliberativeNavigationStop.h"

//Server Actions
#include "robot_control/followWallAction.h"
#include "robot_control/navDriveToAction.h"

//Topics
#include "sensor_msgs/LaserScan.h"
#include "robot_control/localization.h"

#define _USE_MATH_DEFINES
#include <math.h>

typedef actionlib::SimpleActionServer<robot_control::followWallAction> followWallServer;
typedef actionlib::SimpleActionServer<robot_control::navDriveToAction> driveToServer;

/*
*	states = Follow Wall States
*
*	READY  			-> Initial state
* 	STANDBY			-> waiting for next action or transitioning from another one
*  	FOLLOW_WALL		-> going straight
*   AVOID_LEFT		-> Tilt a little to the right without stoping
*   AVOID_RIGHTR	-> Tilt a little to the left without stoping
*   TURN_LEFT		-> Stop and turn left
*   TURN_RIGHT		-> Stop and turn right
*   COLLISION		-> When really near anything*
*
 */
enum states { READY, STANDBY, FOLLOW_WALL, AVOID_LEFT, AVOID_RIGHT, TURN_LEFT, TURN_RIGHT, COLLISION };

/*
*	driveStates = States defined while driving to picking and releasing the product
*  				  or returning to rest
*
*	ROTATING 		-> Rotating towards the goal
* 	DRIVING			-> Driving straight
*  	DONE			-> Achived Goal
*   INIT			-> Initial state
*   WAITING			-> waiting for next action or transitioning from another one
*   ROTATINGLASER	-> Aligning with the product using the laser
*
 */
enum driveStates { ROTATING, DRIVING, DONE, INIT, WAITING, BACKINGUP, ROTATINGLASER };

class Navigator {
public:
	Navigator(ros::NodeHandle node);
	virtual ~Navigator();

	void update();

	//Topics
	void laserHandler(const sensor_msgs::LaserScan::ConstPtr& laserData);
	void localizationHandler(const robot_control::localization::ConstPtr& localization);

	//Services
	bool status(robot_control::deliberativeNavigationStatus::Request &req,
		robot_control::deliberativeNavigationStatus::Response &res);
	bool stop(robot_control::deliberativeNavigationStop::Request &req,
		robot_control::deliberativeNavigationStop::Response &res);

	//Actions
	bool followWall(const robot_control::followWallGoalConstPtr &goal);
	bool driveTo(const robot_control::navDriveToGoalConstPtr &goal);


private:
	enum dists { WALL_MIN = 50, WALL_MAX = 60, SAFE = 40, NEAR_COLLISION = 30, FRONT = 90, DOCKINGMIN = 10};
	
	//Flag for the main loop
	bool active,
	//Flag for following wall
	follow,
	//Flag to choose which wall to follow, true == right
	rightWall,
	//Flag for changing sides when mapping
	sideChange,
	//Action success flag
	success,
	//Flag to know when it starts docking the product
	docking,
	//Flag to set the robot to go backwards a little before following a path
	backup;
	
	//Timer for continuing the same task
	int waitTime,
	//Timer for going backwards a little
	backupTime,
	//Number of regions the laser should be divided
	laserRegions,
	//Stores the size of each regions from the laser (LASER_SIZE/laserRegions)
	regionSize;

	//Storing distances and distance changes
	float distance, initDistance, olddistance;
	//Position and orientation of the robot
	float posx, posy, posz, orientation, fixorientation;
	//General goal when driving between points
	float goalx, goaly;
	//Angle and distance between the robot and the product
	float dockingAngle, productDistance;
	//Timer for updating docking information
	double dockingLaserTime;
	//Distance of the 45 degree laser angle (to the left or right)
	double followHelper;
	//A group of distances that should be verified from each side
	std::vector<double> right, front, left;

	//Drive To Aux variables
	double distg, disto, distgo, diffangle;
	double orix, oriy;

	states robotState, oldState;
	driveStates drivingState, oldDrivingState;

	ros::NodeHandle node;

	ros::Subscriber laser;
	ros::Subscriber localization;

	ros::ServiceClient mapDistanceToMapGridClient;
	ros::ServiceClient baseDriverPowerClient;
	ros::ServiceClient baseDriverRotationClient;

	ros::ServiceServer advertiseStatusService;
	ros::ServiceServer advertiseStopService;

	followWallServer fwServer;
	driveToServer dtServer;

	robot_control::deliberativeNavigationStatus statusSrv;
	robot_control::deliberativeNavigationStop stopSrv;
	robot_control::baseDriverSetDrivePower drivePowerSrv;
	robot_control::baseDriverSetRotation driveRotationSrv;

	robot_control::followWallFeedback fwFeedback;
	robot_control::followWallResult fwResult;
	robot_control::navDriveToFeedback dtFeedback;
	robot_control::navDriveToResult dtResult;
	void postFeedback();

	//Distance verifiers
	bool verifyDistance(std::vector<double> distances, double compare, bool greater);
	void verifyFollowWall();

	//Update functions
	void updateFollowState();

	void updateDriveState();
	void decideRotation();
	void decideRotationDocking();
	void driveForward();
	void driveBackwards();
	void decideVelocity();

	void baseDriverCall();
	void changeDrive(float power = 0.0f, float angle = 0.0f);

	float calcDist(float nposx, float nposy);

	//State and WaitTime getters and setters
	states getState();
	void setState(states newState);
	int getWaitTime();
	void setWaitTime(int time);
};