#include <stdlib.h>
#include <string>
#include <math.h>
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>

/*
* GENERAL GLOBAL VALUES
 */
#define MAPWIDTH 10000
#define MAPHEIGHT 10000
#define CELLSIZE 100
#define MWIDTH MAPWIDTH/CELLSIZE
#define MHEIGHT MAPHEIGHT/CELLSIZE
#define LASERSIZE 640

/*
* SERVICES AND TOPICS
 */

//Application
#define APPLICATIONCREATEMAP			"/Application/createMap"
#define APPLICATIONTRANSPORTPRODUCT		"/Application/transportProduct"
//TransportAgents
#define ROBOT1							"/robot1/"
#define ROBOT2							"/robot2/"

//BaseDriver
#define DRIVEPOWERSERVICE 				"DriveService/setPower"
#define DRIVEROTATIONSERVICE 			"DriveService/setRotation"

//Control
#define CONTROLSTOPSERVICE				"Control/stop"
#define CONTROLREPORTSTATE				"Control/reportState"
#define CONTROLMAPENVIRONMENT			"Control/mapEnvironment"
#define CONTROLDRIVETO					"Control/driveTo"
#define CONTROLRETURNPATHDISTANCE 		"Control/returnPathDistance"
#define CONTROLPICKPRODUCT				"Control/pickProduct"
#define CONTROLRELEASEPRODUCT			"Control/releaseProduct"

//Deliberative Navigation
#define NAVSTATUSSERVICE 				"DeliberativeNavigation/status"
#define NAVSTOPSERVICE 					"DeliberativeNavigation/stop"
#define NAVFOLLOWWALL					"DeliberativeNavigation/followWall"
#define NAVDRIVETO						"DeliberativeNavigation/driveTo"
#define NAVDOCKING						"DeliberativeNavigation/docking"

//Localization
#define ESTIMATEPOSITIONSERVICE			"Localization/estimatePostion"

//Mapping
#define MAPPINGCREATEGRID 				"Mapping/createGridMap"

//MapInformation
#define MAPINFOUPDATEGRID				"/MapInformation/updateGridMap"
#define MAPINFOUPDATEPOSITION			"/MapInformation/updateMapPosition"
#define MAPINFOGETGRID					"/MapInformation/getGridMap"

//Object Manipulation
#define PRODUCTMANIPULATIONGRASP		"ProductManipulation/grasp"
#define PRODUCTMANIPULATIONRELEASE		"ProductManipulation/release"

//PathPlanning
#define PATHPLANNINGDEFINEPATH			"PathPlanning/definePath"

//TransportAgent
#define TRANSPORTAGENTABORT 			"TransportAgent/abort"
#define TRANSPORTAGENTREPORT 			"TransportAgent/report"
#define TRANSPORTAGENTMAPENVIRONMENT 	"TransportAgent/mapEnvironment"
#define TRANSPORTAGENTCOLLECTPRODUCT	"TransportAgent/collecting"
#define TRANSPORTAGENTDELIVERPRODUCT	"TransportAgent/delivering"
#define TRANSPORTAGENTRETURNINGTOREST	"TransportAgent/returningToRest"
#define TRANSPORTAGENTRETURNPATHDISTANCE "TransportAgent/returnPathDistance"

//Topics
#define VELTOPIC 						"Pioneer3AT/cmd_vel"
#define LASERTOPIC 						"Pioneer3AT/laserscan"
#define ODOMTOPIC 						"Pioneer3AT/odom"
#define LOCALIZATIONTOPIC				"Pioneer3AT/localization"
#define TRANSPORTAGENTSUBSCRIBE			"Pioneer3AT/TransportAgent/subscribe"
//Gripper Driver Topics
#define GRIPPERCOMMAND					"Pioneer3AT/gripperCommand"
#define GRIPPERSTATE					"Pioneer3AT/gripperState"
#define GRIPPERSLIDERCOMMAND			"Pioneer3AT/gripperSliderCommand"
#define GRIPPERSLIDERSTATE				"Pioneer3AT/gripperSliderState"

//ROBOT 1 - TransportAgent
#define R1TRANSPORTAGENTABORT 				ROBOT1 TRANSPORTAGENTABORT
#define R1TRANSPORTAGENTREPORT 				ROBOT1 TRANSPORTAGENTREPORT
#define R1TRANSPORTAGENTMAPENVIRONMENT 		ROBOT1 TRANSPORTAGENTMAPENVIRONMENT
#define R1TRANSPORTAGENTCOLLECTPRODUCT		ROBOT1 TRANSPORTAGENTCOLLECTPRODUCT
#define R1TRANSPORTAGENTDELIVERPRODUCT		ROBOT1 TRANSPORTAGENTDELIVERPRODUCT
#define R1TRANSPORTAGENTRETURNINGTOREST		ROBOT1 TRANSPORTAGENTRETURNINGTOREST
#define R1TRANSPORTAGENTRETURNPATHDISTANCE	ROBOT1 TRANSPORTAGENTRETURNPATHDISTANCE
//ROBOT 2 - TransportAgent
#define R2TRANSPORTAGENTABORT 				ROBOT2 TRANSPORTAGENTABORT
#define R2TRANSPORTAGENTREPORT 				ROBOT2 TRANSPORTAGENTREPORT
#define R2TRANSPORTAGENTMAPENVIRONMENT 		ROBOT2 TRANSPORTAGENTMAPENVIRONMENT
#define R2TRANSPORTAGENTCOLLECTPRODUCT		ROBOT2 TRANSPORTAGENTCOLLECTPRODUCT
#define R2TRANSPORTAGENTDELIVERPRODUCT		ROBOT2 TRANSPORTAGENTDELIVERPRODUCT
#define R2TRANSPORTAGENTRETURNINGTOREST		ROBOT2 TRANSPORTAGENTRETURNINGTOREST
#define R2TRANSPORTAGENTRETURNPATHDISTANCE 	ROBOT2 TRANSPORTAGENTRETURNPATHDISTANCE


#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define RESET "\033[0m"