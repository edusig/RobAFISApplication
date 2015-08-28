#include "robot_control/base.h"
#include <robotiq_s_model_articulated_msgs/SModelRobotInput.h>
#include <robotiq_s_model_articulated_msgs/SModelRobotOutput.h>
#include "robot_control/productManipulationGrasp.h"
#include "robot_control/productManipulationRelease.h"
//#include "gripper_slider_msg/SliderCommand.h"
//#include "gripper_slider_msg/SliderState.h"

class ProductManipulation{

public:
	ProductManipulation(ros::NodeHandle);
	virtual ~ProductManipulation();

	bool graspObject(robot_control::productManipulationGrasp::Request &req,
			  	  	 robot_control::productManipulationGrasp::Response &res);
	bool releaseObject(robot_control::productManipulationRelease::Request &req,
			  	  	   robot_control::productManipulationRelease::Response &res);

	void gripperStateHandler(const robotiq_s_model_articulated_msgs::SModelRobotInput::ConstPtr& state);
	//void gripperSliderStateHandler(const gripper_slider_msg::SliderState::ConstPtr& state);

private:
	
	//Gripper

	//Gripper Openess
	// From 0 to 255, measure fingers openess
	uint8_t gripperStatus;

	//Slider
	
	//Slider Status
	//0 = Stopped
	//1 = Moving
	//uint8_t sliderStatus;
	//Slider Result
	//0 = Failed to complete action
	//1 = Action successfull
	//uint8_t sliderResult;

	ros::NodeHandle node;
	robotiq_s_model_articulated_msgs::SModelRobotOutput gripperCommandMsg;
	//gripper_slider_msg::SliderCommand sliderCommandMsg;

	//Topic Subscriber
	ros::Subscriber gripperState;
	//ros::Subscriber sliderState;
	//Topic Publish
	ros::Publisher gripperCommand;
	//ros::Publisher sliderCommand;
	//Service Server
	ros::ServiceServer advertiseGrasp;
	ros::ServiceServer advertiseRelease;

	bool gripperGrasp();
	bool gripperRelease();
	//bool gripperRaise();
	//bool gripperLower();

	void gripperInitialization();

};