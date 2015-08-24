#Services Message Type

##Operation
- driveAgent : Service
- addProductRequest : Service
- subscribe : Topic

##RobAFIS Application
- transportProduct : Action
- createMap : Action

##Transport Agent
- mapEnvironment : Action
- collectProduct : Action
- deliverProduct : Action
- returnToRest : Action
- abort : Service
- report : Service
- subscribe : Topic
- teleoperate : Service

##Control
- driveTo : Action
- pickProduct : Action
- releaseProduct : Action
- mapEnvironment : Action
- stop : Service
- reportState : Service
- subscribe : Topic

##Map Information
- updateGridMap : Service
- updateMapPosition : Service
- getGridMap : Service
- subscribe : Topic

##Deliberative Navigation
- driveTo : Action
- followWall : Action
- executeGlobalPath : Action
- stop : Service
- subscribe : Topic

##Path Planning
- defineGlobalPath : Service

##2D Grid Mapping
- createGridMap : Action
- locationToGrid : Service
- gridToLocation : Service
- subscribe : Topic

##Product Manipulation
- pickProduct : Action
- releaseProduct : Action

##Localization
- estimatePosition : Service
- subscribe : Topic

##Image Processing
- segmentImage : Service
- findColor : Service
- findPattern : Service

##Base Driver
- setDriverPower : Service
- setRotation : Service
- reportState : Service

##GPS Driver
- getPose : Service
- subscribe : Topic
- configure : Service

##Camera Driver
- captureImage : Service
- subscribe : Topic
- configure : Service

##Sonar Driver
- getDistance : Service
- subscribe : Topic
- configure : Service


##Laser Driver
- scan : Service
- subscribe : Topic
- configure : Service


## Gripper Driver
- open : Service
- close : Service
- stop : Service
- reportState : Service

