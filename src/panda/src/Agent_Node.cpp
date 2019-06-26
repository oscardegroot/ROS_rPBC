#include "ros/ros.h"

#include "PandaSim.h"
#include "System.h"
#include "Controller.h"
#include "IDAPBC.h"
#include "CMM.h"
#include "CustomLog.h"

#include <memory>
#include <vector>
#include <string>


int i_id, l, N;

int main(int argc, char **argv){
	
	// Initialise ROS
	ros::init(argc, argv, "Agent");

	ros::NodeHandle n = ros::NodeHandle("~");

	// Get a nodehandle
	n.getParam("i_id", i_id);
	n.getParam("/l_dim", l);
	n.getParam("/N_dim", N);

	
	std::unique_ptr<System> system = std::make_unique<PandaSim>();
	std::unique_ptr<Controller> controller = std::make_unique<IDAPBC>(l, system);

	CMM cmm;

	//Maybe also define the controller here-> Agent has a system and a controller

	ros::Rate loop_rate(100);

	while(ros::ok()){
		// If system data is available

		if(system->dataReady()){
			Eigen::VectorXd tau_network = cmm.sample(controller->getOutput(system));
			//std::cout << "Network Sampled | ";
			Eigen::VectorXd tau = controller->computeControl(system, tau_network);
			//std::cout << "Control Calculated | ";

			system->sendInput(tau);
			//std::cout << "Input send!\n";
			
			system->setDataReady(false);
		}else{
			//system->sendInput(Eigen::VectorXd::Zero(system->n));
			logMsg("Agent Node", "No Data Received From the System!", 1);
		}
		
		
		ros::spinOnce();
		// ros::Duration(0.1).sleep();
		loop_rate.sleep();
	}

	//delete system;
	//delete controller;
	// Not sure if necessary but still
	//delete edges;

	return 0;

}