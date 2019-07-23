cmake_minimum_required(VERSION 2.8.3)
project(panda)

add_compile_options(-std=c++14)
#set(CMAKE_BUILD_TYPE Release) -> apply later!

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
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
  realtime_tools
)


find_package(Franka 0.3.0 REQUIRED)
find_package(Eigen3 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg 
	FILES 
	Waves.msg
)

## Services in the 'srv' folder
add_service_files(DIRECTORY srv
  FILES
  getConnectionsOf.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package(LIBRARIES panda
 CATKIN_DEPENDS 
 roscpp 
 rospy 
 std_msgs 
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
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/classes/Edge.cpp
  src/classes/EdgeDirect.cpp
  src/classes/EdgeDelayFree.cpp
  src/classes/EdgeFlexDelayFree.cpp
  src/classes/Goals.cpp
  src/classes/Remote.cpp
  src/classes/System.cpp
  src/classes/CMM.cpp
  src/classes/PandaSim.cpp
  src/classes/Panda.cpp
  src/classes/IDAPBC.cpp
  src/classes/rPBC.cpp
  src/classes/Controller.cpp
  src/CustomLog.cpp
  src/Helpers.cpp
)


# The agent Node
add_executable(Agent_node src/Agent_Node.cpp)
target_link_libraries(Agent_node ${catkin_LIBRARIES} 
                                 ${PROJECT_NAME}
                                 ${Franka_LIBRARIES})
add_dependencies(Agent_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# add_executable(panda )
# target_link_libraries(panda ${catkin_LIBRARIES} 
#                                  ${PROJECT_NAME}
#                                  ${Franka_LIBRARIES})
# add_dependencies(panda ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(panda
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(panda PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(panda SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(panda PUBLIC
  include
)
#add_dependencies(panda ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



# The beacon node
add_executable(Beacon_node src/Beacon_Node.cpp)
target_link_libraries(Beacon_node ${catkin_LIBRARIES} 
                                 ${PROJECT_NAME}
                                 ${Franka_LIBRARIES})
add_dependencies(Beacon_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# The remote node
add_executable(Remote src/Remote_Node.cpp)
target_link_libraries(Remote ${catkin_LIBRARIES} 
                                 ${PROJECT_NAME}
                                 ${Franka_LIBRARIES})
add_dependencies(Remote ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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
