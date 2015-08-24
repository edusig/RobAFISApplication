#include "robot_control/deliberative_navigation.hh"

/*
	Public Methods
 */

Navigator::Navigator(ros::NodeHandle n): 
	dtServer(n, NAVDRIVETO, boost::bind(&Navigator::driveTo, this, _1), false),
	fwServer(n, NAVFOLLOWWALL, boost::bind(&Navigator::followWall, this, _1), false) {

	node = n;
	robotState = READY;
	oldState = READY;
	drivingState = INIT;
	oldDrivingState = INIT;
	waitTime = 0;
	active = true;
	rightWall = true;
	sideChange = false;
	docking = false;
	fixorientation = -1;
	dockingAngle = -1;
	laserRegions = 64;
	regionSize = (int) floor(640/laserRegions);
	dockingLaserTime = 0;
	dockingLaserTime = ros::Time::now().toSec();

	//Subscribe to laser and odometry topics
	laser = node.subscribe(LASERTOPIC, 100, &Navigator::laserHandler, this);
	localization = node.subscribe(LOCALIZATIONTOPIC, 100, &Navigator::localizationHandler, this);

	//Subscribe to drive services
	baseDriverPowerClient = node.serviceClient<robot_control::baseDriverSetDrivePower>(DRIVEPOWERSERVICE);
	baseDriverRotationClient = node.serviceClient<robot_control::baseDriverSetRotation>(DRIVEROTATIONSERVICE);

	//Advertise its own service
	advertiseStatusService = node.advertiseService(NAVSTATUSSERVICE, &Navigator::status, this);
	advertiseStopService = node.advertiseService(NAVSTOPSERVICE, &Navigator::stop, this);

	fwServer.start();
	dtServer.start();
}

Navigator::~Navigator() {
	changeDrive();
	baseDriverCall();
}

void Navigator::update(){
	if(active){

		updateFollowState();

		if(robotState != oldState){

			oldState = robotState;

			decideVelocity();
			baseDriverCall();
		}
	
	}
}

void Navigator::laserHandler(const sensor_msgs::LaserScan::ConstPtr& laserData){

	if(laserData != NULL){
		right.clear();
		front.clear();
		left.clear();
		int x;
		for(x = 0; x < 3; x++){
			right.push_back(laserData->ranges[0+(10*x)]);
			front.push_back(laserData->ranges[310+(10*x)]);
			left.push_back(laserData->ranges[619+(10*x)]);
		}

		if(docking){

			double timedelta = ros::Time::now().toSec() - dockingLaserTime;

			if(timedelta > 0.5){
				dockingLaserTime = ros::Time::now().toSec();			
				float ranges[laserRegions], min = 0.0f;
				int index, mindex;
				std::fill(ranges, ranges+laserRegions, 0.0f);
				for(x = 0; x < 638; x+=2){
					//Divides the laser in n regions of about 640/n degrees
					index = floor(x/regionSize);
					ranges[index] += laserData->ranges[x];
				}
				for(x = 0; x < laserRegions; x++){
					ranges[x] /= regionSize;
					if(min == 0 || ranges[x] < min){
						min = ranges[x];
						mindex = x;
					}
				}
				min *= 10;
				//If the minimum measured distance (mean of a region) is within the distance measured when there is a product ahead
				if(min < DOCKINGMIN){
						//Calculate the center angle of the chosen region
						dockingAngle = ((mindex * regionSize) + (regionSize/2)) * 0.28125;
						productDistance = min;
				}else{
					dockingAngle = -1;
					productDistance = -1;
				}
				//ROS_INFO("Navigation: product distance = %.2f", productDistance);
				//ROS_INFO("Navigation: docking angle = %.2f", dockingAngle);
			}
		}

		//Aproximadamente 45' / 479 comes from 639(laser size) - 160 when following the left wall
		followHelper = rightWall?laserData->ranges[160]:laserData->ranges[479];

	}

}

//Updates product localization
void Navigator::localizationHandler(const robot_control::localization::ConstPtr& localization){
	posx = localization->posx;
	posy = localization->posy;
	posz = localization->posz;
	orientation = localization->orientation;
	if(fixorientation == -1)
		fixorientation = orientation + (1.57/2);
}

//TODO: Implement this with transport agent and control
bool Navigator::status(robot_control::deliberativeNavigationStatus::Request &req,
					   robot_control::deliberativeNavigationStatus::Response &res){
	return true;
}

//TODO: Call this from control
bool Navigator::stop(robot_control::deliberativeNavigationStop::Request &req,
					 robot_control::deliberativeNavigationStop::Response &res){
	changeDrive();
	baseDriverCall();
	res.success = true;
	return true;

}

bool Navigator::followWall(const robot_control::followWallGoalConstPtr &goal){
	follow = true;
	active = true;
	success = true;

	ros::Rate r(20);

	//Signal the change in the wall it should be following
	if(rightWall != goal->rightWall)
		sideChange = true;

	rightWall = goal->rightWall;
	oldState = READY;
	goalx = goal->posx;
	goaly = goal->posy;
	//Euclidian distance between goalx,goaly and posx,posy
	initDistance = calcDist(goalx, goaly);
	olddistance = initDistance;

	while(follow && active){
		//Update robot's follow wall state
		update();
		//Post goal progression feedback
		postFeedback();

		//Check for a preempt from the client
		//ROS Wiki - 1.2.1 line 48-55 
		//(http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29)
		if(fwServer.isPreemptRequested() || !ros::ok()){
			fwServer.setPreempted();
			success = false;
			break;
		}

		r.sleep();
	}
	changeDrive();
	baseDriverCall();

	fwResult.success = success;
	if(success)
		fwServer.setSucceeded(fwResult);
	else
		fwServer.setPreempted(fwResult);
	return success;
}

void Navigator::postFeedback(){

	distance = calcDist(goalx, goaly);
	if(distance < 0.5)
		follow = false;
	if(fabs(distance - olddistance) > 0.1f){
		olddistance = distance;
		fwFeedback.progress = 100 - ((distance/initDistance) * 100);
		fwServer.publishFeedback(fwFeedback);
	}

}

//Quando greater é verdadeiro ele verifica se todas as ranges são maiores
//Quando greater é false ele verifica se pelo menos 1 das ranges é menor
//É feito assim, pois quando se quer verificar se há espaco em algum dos lados
//todas as ranges tem que apontar como espaco vazio.
//E quando se quer verificar se esta perto da parede se pelo menos uma das ranges 
//estiver perto de parede já tem parede
bool Navigator::verifyDistance(std::vector<double> distances, double compare, bool greater){
	int x;
	double realCompare = compare / 100;
	bool flag = greater;
	//TODO: Simplify using bitwise operators
	for(x = 0; x < distances.size(); x++){
		if(greater)
			flag = distances[x] > realCompare?flag:false;
		else
			flag = distances[x] < realCompare?true:flag;
	}
	
	return flag;
}

//Maintain the robot near the wall, using a triangulation between 2 laser rays (0 and 45).
//Combining the 2 rays you make a triangle with the wall and can calculate the distance from
//it even when not totally aligned.
void Navigator::verifyFollowWall(){
	double hypotenuse = followHelper;
	double side, wall_side;

	if(left.size() <= 0 || right.size() <= 0)
		return;

	side = rightWall?right[0]:left[2];
	wall_side = sqrt( pow(hypotenuse, 2) - pow(side, 2) );

	//Transforma em cm para comparar com a distancia do enum de distancias
	//TODO: Use mapping service conversion
	wall_side *= 100;

	if(wall_side < WALL_MIN)
		setState(rightWall?AVOID_RIGHT:AVOID_LEFT);
	else if(wall_side > WALL_MAX)
		setState(rightWall?AVOID_LEFT:AVOID_RIGHT);
	else
		setState(FOLLOW_WALL);

	setWaitTime(2);
}

void Navigator::updateFollowState(){

	if(sideChange){
		setState(rightWall?TURN_LEFT:TURN_RIGHT);
		setWaitTime(7);
		sideChange = false;
		return;
	}

	if(verifyDistance(right, NEAR_COLLISION, false) || verifyDistance(front, NEAR_COLLISION+15, false) || verifyDistance(left, NEAR_COLLISION, false)){
		setState(COLLISION);		
		return;
	}

	if(waitTime > 0){
		waitTime--;
		return;
	}

	if(verifyDistance(front, FRONT, false)){
		setState(rightWall?TURN_LEFT:TURN_RIGHT);
		setWaitTime(5);
	}else{
		verifyFollowWall();
	}

}

void Navigator::decideVelocity(){

	if(robotState != COLLISION)
		changeDrive();

	switch(robotState){
		case TURN_LEFT:
			changeDrive(0.0, 0.3);
			break;
		case TURN_RIGHT:
			changeDrive(0.0, -0.3);
			break;
		case COLLISION:
			changeDrive(-0.3);
			break;
		case AVOID_RIGHT:
			changeDrive(0.3, 0.3);
			break;
		case AVOID_LEFT:
			changeDrive(0.3, -0.3);
			break;
		case FOLLOW_WALL:
			changeDrive(0.5);
			break;
		case STANDBY:
		default:
			changeDrive();
			break;
	}
}

bool Navigator::driveTo(const robot_control::navDriveToGoalConstPtr &goal){

	double distg, disto, distgo, diffangle;
	double orix, oriy, rotation;

	active = true;
	success = false;
	goalx = goal->posx;
	goaly = goal->posy;
	docking = goal->docking;
	backup = goal->backup;

	if(backup)
		backupTime = 300;

	//ROS_INFO("Navigation: Driving to %.2f %.2f", goalx, goaly);
	//ROS_INFO("Navigation: Current Location %.2f %.2f", posx, posy);

	ros::Rate r(200);
	initDistance = -1;
	olddistance = -1;

	drivingState = INIT;
	oldDrivingState = INIT;

	while(active){

		updateDriveState();

		if(drivingState != oldDrivingState){
			ROS_INFO("Navigation: Driving state changed to = %d", drivingState);
			switch(drivingState){
				case ROTATING:
					decideRotation();
					break;
				case ROTATINGLASER:
					decideRotationDocking();
					break;
				case DRIVING:
					driveForward();
					break;
				case BACKINGUP:
					ROS_INFO("Navigation: going backwards!");
					driveBackwards();
					break;
				case WAITING:
					break;
				case DONE:
					active = false;
					success = true;
					break;
			}
		}

		oldDrivingState = drivingState;

		if(dtServer.isPreemptRequested() || !ros::ok()){
			dtServer.setPreempted();
			success = false;
			active = false;
			break;
		}

		r.sleep();
	}

	//Stops robot
	changeDrive();
	baseDriverCall();

	dtResult.success = success;
	if(success)
		dtServer.setSucceeded(dtResult);
	else
		dtServer.setPreempted(dtResult);
	return success;

}

void Navigator::updateDriveState(){

	if(backupTime > 0){
		backupTime--;
		drivingState = BACKINGUP;
		return;
	}

	/*
	*	Using the law of cosine we can calculate the angle between where the robot is oriented at
	*	and the goal point it must turn to.
	* 	See more at: https://en.wikipedia.org/wiki/Law_of_cosines	
	*
	 */

	//Calculates the position where the robot is oriented using the distance of the goal
	orix = posx + (distg * cos(orientation));
	oriy = posy + (distg * sin(orientation));

	//Distance between robot and goal point
	distg = sqrt(pow(posx - goalx, 2) + pow(posy - goaly, 2)); //P12 A
	//Distance between the point the robot is oriented at and the position he is at.
	disto = sqrt(pow(orix - posx, 2) + pow(oriy - posy, 2)); //P13 B
	//Distance between the point the robot is oriented at and the goal position.
	distgo = sqrt(pow(goalx - orix, 2) + pow(goaly - oriy, 2)); //P23 C

	//Calculated difference in angle
	diffangle = acos( (pow(distg,2) + pow(disto,2) - pow(distgo,2)) / (2 * distg * disto) );

	distance = distg;
	if(initDistance < 0)
		initDistance = distance;


	//Publish progress when some distance change
	if(fabs(distance - olddistance) > 0.2){
		olddistance = distance;
		dtFeedback.progress = 100 - ((distance/initDistance) * 100);
		dtServer.publishFeedback(dtFeedback);
	}

	//When near the goal and found product start docking actions
	if(docking && distg < 0.6 && productDistance > 0 && productDistance < DOCKINGMIN){
		if(fabs(dockingAngle-90) > 5){
			drivingState = ROTATINGLASER;
			return;
		}else if(productDistance <= 1){
			drivingState = DONE;
			changeDrive();
			baseDriverCall();
			return;
		}else{
			drivingState = DRIVING;
			return;
		}
	}

	if(diffangle > 0.2f)
		drivingState = ROTATING;
	else if(distg > 0.3f)
		drivingState = DRIVING;
	else
		drivingState = DONE;

}

void Navigator::driveForward(){

	float vel = docking ? 0.15 : 0.4;
	if(docking && productDistance < 1.5)
		vel /= 3;
	changeDrive(vel, 0);
	baseDriverCall();

	if(initDistance == -1 || olddistance == -1){
		initDistance = distance;
		olddistance = distance;
	}

}

void Navigator::driveBackwards(){
	changeDrive(-0.4, 0);
	baseDriverCall();
}

//Calculates which side to turn minimizing the difference in angle
void Navigator::decideRotation(){

	double adjustleft, adjustright;
	double lposx, lposy, rposx, rposy;
	double distx, disty;
	double distl, distr, distg;
	float vel;
	
	adjustleft = orientation + 0.1;
	adjustright = orientation - 0.1;

	distx = fabs(goalx - posx);
	disty = fabs(goaly - posy);
	
	lposx = posx + (distx * cos(adjustleft));
	lposy = posy + (disty * sin(adjustleft)); 
	
	rposx = posx + (distx * cos(adjustright));
	rposy = posy + (disty * sin(adjustright));

	distl = sqrt(pow(goalx - lposx, 2) + pow(goaly - lposy, 2));
	distr = sqrt(pow(goalx - rposx, 2) + pow(goaly - rposy, 2));

	vel = docking ? 0.15 : 0.25;
	vel = distl > distr ? -vel : vel;
	changeDrive(0, vel);
	baseDriverCall();

}

void Navigator::decideRotationDocking(){
	float rotate = dockingAngle > 90 ? 0.03 : -0.03;
	changeDrive(0.0, rotate);
	baseDriverCall();
}

void Navigator::baseDriverCall(){
	//Calling base driver service to change velocities
	if(!baseDriverPowerClient)
		baseDriverPowerClient = node.serviceClient<robot_control::baseDriverSetDrivePower>(DRIVEPOWERSERVICE);
	baseDriverPowerClient.call(drivePowerSrv);

	//Calling base driver service to change rotation			
	if(!baseDriverRotationClient)
		baseDriverRotationClient = node.serviceClient<robot_control::baseDriverSetRotation>(DRIVEROTATIONSERVICE);
	baseDriverRotationClient.call(driveRotationSrv);

}

void Navigator::changeDrive(float power, float angle){
	drivePowerSrv.request.power = power;
	driveRotationSrv.request.angle = angle;
}

float Navigator::calcDist(float nposx, float nposy){
	return fabs(posx - nposx) + fabs(posy - nposy);
}

states Navigator::getState(){
	return robotState;
}

void Navigator::setState(states newState){
	robotState = newState;
}

int Navigator::getWaitTime(){
	return waitTime;
}

void Navigator::setWaitTime(int time){
	waitTime = time;
}


int main(int argc, char **argv){

	ros::init(argc, argv, "Navigator");

	ros::NodeHandle node;

	Navigator *nav = new Navigator(node);

	ROS_INFO("Navigator Running!!!");

	ros::spin();
}

