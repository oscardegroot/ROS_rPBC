#include "ros/ros.h"

#include "PandaSim.h"
#include "System.h"
#include "Controller.h"
#include "IDAPBC.h"
#include "rPBC.h"
#include "CMM.h"
#include "CustomLog.h"

#include <memory>
#include <vector>
#include <string>


int id, l, N;

int main(int argc, char **argv){
	
	// Initialise ROS
	ros::init(argc, argv, "Agent");

	ros::NodeHandle nh = ros::NodeHandle("~");

	// Get a nodehandle
	helpers::safelyRetrieve(nh, "ID", id);
	helpers::safelyRetrieve(nh, "/l", l);
	helpers::safelyRetrieve(nh, "/N_agents", N);

	
	std::unique_ptr<System> system = std::make_unique<PandaSim>();
	std::unique_ptr<Controller> controller = std::make_unique<IDAPBC>(*system);
    system->setAgent(nh, "pandasim");
    
	CMM cmm(id, system->agent.sampling_rate);

	ros::Rate loop_rate(1000);

	while(ros::ok()){
        
        Eigen::VectorXd tau_network = cmm.sample(controller->getOutput(*system));
        //std::cout << "Network Sampled | ";
        Eigen::VectorXd tau = controller->computeControl(*system, tau_network);
        //std::cout << "Control Calculated | ";

        system->sendInput(tau);
        //std::cout << "Input send!\n";
		
		ros::spinOnce();
		// ros::Duration(0.1).sleep();
		loop_rate.sleep();
	}


	return 0;

}