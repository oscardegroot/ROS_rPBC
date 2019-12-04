cmake_minimum_required(VERSION 2.8.3)
project(panda)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)# -> apply later!

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
	std_srvs
  sensor_msgs
  controller_interface
  dynamic_reconfigure
  franka_hw
  franka_control
  geometry_msgs
  hardware_interface
  visualization_msgs
  message_generation
  pluginlib
  tf
  cv_bridge
  realtime_tools
)


find_package(Franka 0.4.0 REQUIRED)
find_package(Eigen3 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg 
	FILES 
	Waves.msg
	Move.msg
	Readout.msg
)

## Services in the 'srv' folder
add_service_files(DIRECTORY srv
  	FILES
  	getConnectionsOf.srv
	registerElisa3.srv
	colorElisa3.srv
	isAgentLeader.srv
	registerAgent.srv
	addObstacle.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package(LIBRARIES panda
	CATKIN_DEPENDS
	roscpp
	rospy
	std_msgs
	std_srvs
	sensor_msgs
	visualization_msgs
	message_runtime
	controller_interface
	dynamic_reconfigure
	franka_control
	franka_hw
	geometry_msgs
	hardware_interface
	pluginlib
	realtime_tools
	DEPENDS Franka
)
 
###########
## Build ##
###########

# Add libraries
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	#${OpenCV_INCLUDE_DIRS} 
	src/pc-side-elisa3-library
)

# Add the Elisa3 Library Directory
link_directories(src/pc-side-elisa3-library/linux)

## Declare a C++ library
add_library(${PROJECT_NAME}
	src/classes/Edge.cpp
	src/classes/EdgeDirect.cpp
	src/classes/EdgeFlex.cpp
	src/classes/EdgeLeader.cpp
	src/classes/Agent.cpp
	src/classes/EdgeDelayFree.cpp
	src/classes/EdgeFlexDelayFree.cpp
	src/classes/Goals.cpp
	src/classes/AgentStation.cpp
	src/classes/ElisaStation.cpp
	src/classes/System.cpp
	src/classes/CMM.cpp
	src/classes/Elisa3.cpp
	src/classes/PandaSim.cpp
	src/classes/ConnectedPandasim.cpp
	src/classes/Potential.cpp
	src/classes/Obstacle.cpp
	src/classes/Goal.cpp
	src/classes/Panda.cpp
	src/classes/Selector.cpp
	src/classes/IDAPBC.cpp
 	src/classes/rPBC.cpp
 	src/classes/Formation.cpp
	src/pc-side-elisa3-library/elisa3-lib.c
	src/pc-side-elisa3-library/usb-comm.c
	src/classes/Controller.cpp
	src/CustomLog.cpp
	src/Helpers.cpp
)

# Elisa Node
add_executable(Elisa3_Node src/Elisa3_Node.cpp)
target_link_libraries(Elisa3_Node ${catkin_LIBRARIES} ${PROJECT_NAME} elisa3 pthread usb-1.0)

# The agent Node
add_executable(Agent_node src/Agent_Node.cpp)
target_link_libraries(Agent_node ${catkin_LIBRARIES}
	 ${PROJECT_NAME}
	 ${Franka_LIBRARIES}
)
add_dependencies(Agent_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(NetworkNode src/NetworkNode.cpp)
target_link_libraries(NetworkNode ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(NetworkNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(NetworkNode SYSTEM PUBLIC
		${catkin_INCLUDE_DIRS}
)

# The Panda Node
add_dependencies(panda
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(panda SYSTEM PUBLIC
		${Franka_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIRS}
		${catkin_INCLUDE_DIRS}
)
target_link_libraries(panda PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
	elisa3
	pthread
	usb-1.0
)


target_include_directories(panda PUBLIC
  include
)

# The text interface
add_executable(TextInterface_Node src/TextInterface.cpp)
target_link_libraries(TextInterface_Node ${catkin_LIBRARIES}
		${PROJECT_NAME}
		${Franka_LIBRARIES})
add_dependencies(TextInterface_Node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# The beacon node
add_executable(Beacon_node src/Beacon_Node.cpp)
target_link_libraries(Beacon_node ${catkin_LIBRARIES}
                                 ${PROJECT_NAME}
                                 ${Franka_LIBRARIES})
add_dependencies(Beacon_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# The remote node
add_executable(Server src/ServerNode.cpp)
target_link_libraries(Server ${catkin_LIBRARIES}
                                 ${PROJECT_NAME}
                                 ${Franka_LIBRARIES})
add_dependencies(Server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

###################################################################################

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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_panda.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
