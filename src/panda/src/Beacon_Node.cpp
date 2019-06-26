/* 
File: Beacon_Node.cpp

A beacon is a system that only contains a CMM such that it may influence convergence of other systems without having actual dynamics.
Thanks to the CMM the communication and convergence properties remain stable.

*/



#include "ros/ros.h"

#include "panda/Waves.h"
#include "std_msgs/Float64MultiArray.h"
#include "panda/getConnectionsOf.h"

#include "Goals.h"
#include "CMM.h"

#include <memory>
#include <vector>
#include <string>
#include <sstream>

int i_id, l, N;

int main(int argc, char **argv){
	
	// Initialise ROS
	ros::init(argc, argv, "Agent");

	ros::NodeHandle n = ros::NodeHandle("~");

	// Get a nodehandle
	n.getParam("i_id", i_id);
	n.getParam("/l_dim", l);
	n.getParam("/N_dim", N);
	
	Eigen::Vector3d ref;
	ref << 0.3, 0.3, 0.4;

	CMM cmm;

	//Maybe also define the controller here-> Agent has a system and a controller

	ros::Rate loop_rate(100);

	while(ros::ok()){

		// Sample the network
		Eigen::VectorXd tau_network = cmm.sample(ref);
		
		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;

}