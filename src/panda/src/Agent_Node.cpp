#include "ros/ros.h"

#include "panda/Waves.h"
#include "std_msgs/Float64MultiArray.h"
#include "panda/getConnectionsOf.h"

#include "Agent.h"
#include "Panda.h"
#include "Edge.h"
#include "Goals.h"

#include <vector>
#include <string>
#include <sstream>


int main(int argc, char **argv){
	
	// Initialise ROS
	ros::init(argc, argv, "Agent");

	Panda panda;
	//Maybe also define the controller here.

	ros::Rate loop_rate(0.5);

	while(ros::ok()){
		
		Eigen::VectorXd tau_i = panda.sample();
		std::cout << "Tau: " << tau_i << "\n";
		
		ros::spinOnce();
		// ros::Duration(0.1).sleep();
		loop_rate.sleep();
	}

	// Not sure if necessary but still
	//delete edges;

	return 0;

}