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

    std::string output;
    helpers::safelyRetrieve(nh, "/output", output);

	// Initialise the system and controller
	std::unique_ptr<System> system = std::make_unique<PandaSim>();
    std::unique_ptr<Controller> controller;
    if(output == "z"){
         controller = std::make_unique<IDAPBC>(*(system->cmm->agent));
    }else{
         controller = std::make_unique<rPBC>(*(system->cmm->agent));
    }

    // Perform handshake after controller initialisation!
    system->cmm->performHandshake();

	ros::Rate loop_rate(1000);

	while(ros::ok()){
        
        PROFILE_SCOPE("Agent Loop");
        
        // Sample the network
        Eigen::VectorXd tau_network = system->cmm->sample(controller->getOutput(*system));

        // Compute the input
        Eigen::VectorXd tau = controller->computeControl(*system, tau_network);

        // Implement the input
        system->sendInput(tau);
		
        // Check callbacks and sleep
		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;

}