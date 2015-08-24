cmake_minimum_required(VERSION 2.8.3)
project(robot_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  genmsg
  actionlib_msgs
  actionlib
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  localization.msg
)

## Generate services in the 'srv' folder
add_service_files(
	FILES
  #baseDriverReportState.srv
  baseDriverSetDrivePower.srv
  baseDriverSetRotation.srv
  controlReportState.srv
  controlReturnPathDistance.srv
  controlStop.srv
  deliberativeNavigationStatus.srv
  deliberativeNavigationStop.srv
  gridMappingCreateMap.srv
  gridMappingGridToLocation.srv
  gridMappingLocationToGrid.srv
  localizationEstimatePosition.srv
  mapInformationGetGridMap.srv
  mapInformationUpdateGridMap.srv
  mapInformationUpdateMapPosition.srv
  productManipulationGrasp.srv
  productManipulationRelease.srv
  pathPlanningDefineGlobalPath.srv
  robAFISApplicationTransportProduct.srv
  #robAFISApplicationReport.srv
  transportAgentAbort.srv
  transportAgentReport.srv
  transportAgentReturnPathDistance.srv
  #transportAgentTeleoperate.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  collectProduct.action
  createMap.action
  deliverProduct.action
  docking.action
  driveTo.action
  followWall.action
  mapEnvironment.action
  navDriveTo.action
  pickProduct.action
  releaseProduct.action
  returnToRest.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_control
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} include
  src/libraries
)

## Declare a cpp library
# add_library(robot_control
#   src/${PROJECT_NAME}/robot_control.cpp
# )
add_library(OGMap src/libraries/OGMap.cc)


## Declare a cpp executable
# add_executable(robot_control_node src/robot_control_node.cpp)
add_executable(application src/application.cpp)
add_executable(base_driver src/base_driver.cpp)
add_executable(control src/control.cpp)
add_executable(deliberative_navigation src/deliberative_navigation.cpp)
add_executable(grid_mapping src/grid_mapping.cpp)
add_executable(localization src/localization.cpp)
add_executable(map_information src/map_information.cpp)
add_executable(operation src/operation.cpp)
add_executable(path_planning src/path_planning.cpp)
add_executable(product_manipulation src/product_manipulation.cpp)
add_executable(transport_agent src/transport_agent.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(robot_control_node robot_control_generate_messages_cpp)
add_dependencies(application robot_control_generate_messages_cpp)
add_dependencies(base_driver robot_control_generate_messages_cpp)
add_dependencies(control robot_control_generate_messages_cpp)
add_dependencies(deliberative_navigation robot_control_generate_messages_cpp)
add_dependencies(grid_mapping robot_control_generate_messages_cpp)
add_dependencies(localization robot_control_generate_messages_cpp)
add_dependencies(map_information robot_control_generate_messages_cpp)
add_dependencies(operation robot_control_generate_messages_cpp)
add_dependencies(path_planning robot_control_generate_messages_cpp)
add_dependencies(product_manipulation robot_control_generate_messages_cpp)
add_dependencies(transport_agent robot_control_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(application ${catkin_LIBRARIES})
target_link_libraries(base_driver ${catkin_LIBRARIES})
target_link_libraries(control ${catkin_LIBRARIES})
target_link_libraries(deliberative_navigation ${catkin_LIBRARIES})
target_link_libraries(grid_mapping ${catkin_LIBRARIES})
target_link_libraries(localization ${catkin_LIBRARIES})
target_link_libraries(map_information ${catkin_LIBRARIES})
target_link_libraries(operation ${catkin_LIBRARIES})
target_link_libraries(path_planning ${catkin_LIBRARIES})
target_link_libraries(path_planning OGMap)
target_link_libraries(product_manipulation ${catkin_LIBRARIES})
target_link_libraries(transport_agent ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS robot_control robot_control_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
