#include "robot_control/product_manipulation.hh"

ProductManipulation::ProductManipulation(ros::NodeHandle n){
	node = n;
	gripperInitialization();
	//Create a Publisher to Gripper Command Topic
	gripperCommand = node.advertise<robotiq_s_model_articulated_msgs::SModelRobotOutput>(GRIPPERCOMMAND, 1000);
	//Create a Publisher to Gripper Slider Command Topic
	//sliderCommand = node.advertise<gripper_slider_msg::SliderCommand>(GRIPPERSLIDERCOMMAND, 1000);

	//Subscribe to Gripper Status
	gripperState = node.subscribe(GRIPPERSTATE, 100, &ProductManipulation::gripperStateHandler, this);
	//Subscribe to Gripper Slider Status
	//sliderState = node.subscribe(GRIPPERSLIDERSTATE, 100, &ProductManipulation::gripperSliderStateHandler, this);

	//Advertise Controls
	advertiseGrasp = node.advertiseService(PRODUCTMANIPULATIONGRASP, &ProductManipulation::graspObject, this);
	advertiseRelease = node.advertiseService(PRODUCTMANIPULATIONRELEASE, &ProductManipulation::releaseObject, this);

}

ProductManipulation::~ProductManipulation(){}

//Services Advertised
bool ProductManipulation::graspObject(robot_control::productManipulationGrasp::Request &req,
			  	  					 robot_control::productManipulationGrasp::Response &res){

	res.success = false;

	//Grasp Object
	gripperGrasp();

	//Raise Gripper
/*	gripperRaise();
*/
	res.success = true;
	return true;
}

bool ProductManipulation::releaseObject(robot_control::productManipulationRelease::Request &req,
			  	  					   robot_control::productManipulationRelease::Response &res){

	//Lower Gripper
	/*
	gripperLower();
*/
	//Release Object
	gripperRelease();
	res.success = true;
	return true;
}

//Gripper State Handler
void ProductManipulation::gripperStateHandler(const robotiq_s_model_articulated_msgs::SModelRobotInput::ConstPtr& state){

	gripperStatus = state->gPRA;

}

/*
void ProductManipulation::gripperSliderStateHandler(const gripper_slider_msg::SliderState::ConstPtr& state){
	sliderStatus = state->moving;
	sliderResult = state->success;
}
*/
//Helper Functions to control the gripper

//Gripper Initialization
//Initialize it open
void ProductManipulation::gripperInitialization(){
	gripperCommandMsg.rACT = 1;
	gripperCommandMsg.rMOD = 0;
	gripperCommandMsg.rGTO = 1;
	gripperCommandMsg.rATR = 0;
	gripperCommandMsg.rICF = 0;
	gripperCommandMsg.rICS = 0;
	//gripperCommandMsg.rPRA = 0;
	gripperCommandMsg.rSPA = 255;
	gripperCommandMsg.rFRA = 0;
	gripperCommandMsg.rPRB = 155;
	gripperCommandMsg.rSPB = 0;
	gripperCommandMsg.rFRB = 0;
	gripperCommandMsg.rPRC = 255;
	gripperCommandMsg.rSPC = 0;
	gripperCommandMsg.rFRC = 0;
	gripperCommandMsg.rPRS = 0;
	gripperCommandMsg.rSPS = 0;
	gripperCommandMsg.rFRS = 0;

}

//Robotiq Gripper Open and Close
bool ProductManipulation::gripperGrasp(){
	//{1,3,1,0,0,0,255,255,0,155,0,0,255,0,0,0,0,0} Robotiq scissor close
	ROS_INFO("ProductManipulation: Publishing Grasp Command!");
	ROS_INFO("ProductManipulation: Gripper status = %d", gripperStatus);
	gripperCommandMsg.rPRA = 60;
	gripperCommand.publish(gripperCommandMsg);
	ros::Duration(3).sleep();
	ROS_INFO("ProductManipulation: Gripper status = %d", gripperStatus);
	gripperCommandMsg.rPRA = 110;
	gripperCommand.publish(gripperCommandMsg);
	ros::Duration(1).sleep();
	ROS_INFO("ProductManipulation: Gripper status = %d", gripperStatus);
}

bool ProductManipulation::gripperRelease(){
	//{1,2,1,0,0,0,0,255,0,155,0,0,255,0,0,0,0,0} Robotiq open full wide
	ROS_INFO("ProductManipulation: Publishing Release Command!");
	ROS_INFO("ProductManipulation: Gripper status = %d", gripperStatus);
	gripperCommandMsg.rPRA = 60;
	gripperCommand.publish(gripperCommandMsg);
	ros::Duration(3).sleep();
	ROS_INFO("ProductManipulation: Gripper status = %d", gripperStatus);
	gripperCommandMsg.rPRA = 0;
	gripperCommand.publish(gripperCommandMsg);
	ros::Duration(1).sleep();
	ROS_INFO("ProductManipulation: Gripper status = %d", gripperStatus);
	gripperCommandMsg.rPRA = 0;
	gripperCommand.publish(gripperCommandMsg);
	ros::Duration(1).sleep();
	ROS_INFO("ProductManipulation: Gripper status = %d", gripperStatus);
}

//Gripepr custom slider Raise and Lower
/*
bool ProductManipulation::gripperRaise(){
	sliderCommandMsg.raise = true;
	sliderCommand.publish(sliderCommandMsg);
}

bool ProductManipulation::gripperLower(){
	sliderCommandMsg.raise = false;
	sliderCommand.publish(sliderCommandMsg);
}
*/
/*
*
*
*	Main
*
*
*/

int main(int argc, char **argv){

	ros::init(argc, argv, "object_manipulation");

	ros::NodeHandle node;

	ProductManipulation *objmanipulator = new ProductManipulation(node);

	ROS_INFO("Object Manipulation Running!!!");

	ros::spin();
}