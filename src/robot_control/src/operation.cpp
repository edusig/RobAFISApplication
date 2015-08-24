#include "robot_control/operation.hh"

Operation::Operation(ros::NodeHandle n): client(APPLICATIONCREATEMAP, true){
	node = n;
	op = 3;
	active = true;
	mapped = false;
	//Connect to client service
	transportProductClient = node.serviceClient<robot_control::robAFISApplicationTransportProduct>(APPLICATIONTRANSPORTPRODUCT);
	//Wait for the service to connect
	client.waitForServer();
}

Operation::~Operation(){}

//Calls Client Action to start mapping the environment
void Operation::createMap(){
	goal.active = true;
	//Send goal to action client
	client.sendGoal(goal, 
		boost::bind(&Operation::createMapFinish, this, _1, _2), //Finish action handler
		createMapClient::SimpleActiveCallback(),  //Activate action handler
		boost::bind(&Operation::createMapUpdateHandler, this, _1)); //Update action handler
	active = false;
}

//Show mapping progress
void Operation::createMapUpdateHandler(const robot_control::createMapFeedbackConstPtr& feedback){
	ROS_INFO("Progress: %.1f%%", feedback->progress);
}

//Handle create map action finish
//State 	-> action client state
//Result 	-> result struct (check ../action/createMap.action)
void Operation::createMapFinish(const actionlib::SimpleClientGoalState& state,
								const robot_control::createMapResultConstPtr& result){
	printf("Operation Current State: %s\n", client.getState().toString().c_str());
	ROS_INFO("Map Created");
	active = true;
}

//Send product request to Application
//product1 	-> true = pick product 1 | false = pick product 2
void Operation::addProductRequest(bool product1){

	//Set product1 in the transportProduct struct
	transportProductSrv.request.product1 = product1;

	//If service client is not set, or disconnected, reconnect
	if(!transportProductClient)
		transportProductClient = node.serviceClient<robot_control::robAFISApplicationTransportProduct>(APPLICATIONTRANSPORTPRODUCT);

	//Call service with its struct
	transportProductClient.call(transportProductSrv);

}

//Print the intil menu options on the screen and wait for the response
void Operation::userInterface(){
	printf("Menu:\n");
	printf("0 - Exit | 1 - CreateMap | 2 - AddProduct\n");
	scanf("%d", &op);
}

//Main loop
void Operation::update(){
	if(active){
		//Show initial interface
		userInterface();
		switch(op){
			case 0: //Exit
				active = false;
				ros::shutdown();
				break;
			case 1: //Create Map
				if(!mapped){
					createMap();
					mapped = true;
				}else
					printf("Already mapped!\n");
				break;
			case 2: //Add Product Request
				//Cannot add product before mapping the environment
				if(!mapped){
					printf("Map the environment before adding a produt request!\n");
					break;
				}
				int prod; //selected product
				printf("Which product?\n Product (1) at (5.0 2.5) or Product (2) at (5.0 7.5)\n");
				scanf("%d", &prod);
				if(prod < 0 || prod > 2){
					printf("Invalid product!");
					return;
				}else if(prod == 0){
					op = 3;
					return;
				}
				addProductRequest(prod==1);
				break;
			case 3: //Waiting
				break;
			default:
				printf("Invalid operation!\n");
				break;
		}
	}
}

int main(int argc, char **argv){

	ros::init(argc, argv, "operation");
	ros::NodeHandle node;
	ros::Rate loop_rate(100); //Define loop to execute 100 times/sec

	Operation *operation = new Operation(node);

	ROS_INFO("Operation Running!!!");

	while(ros::ok()){
		//Calls main loop
		operation->update();

		ros::spinOnce();
		loop_rate.sleep();

	}

}