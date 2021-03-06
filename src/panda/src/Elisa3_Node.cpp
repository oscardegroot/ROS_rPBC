/*
 * File: Elisa3_Node.cpp
 *
 * Node that runs an Elisa3 robot. Main functionality implemented in the Elisa3 class.
 */


#include "ros/ros.h"
#include "Helpers.h"
#include "Elisa3.h"
#include "System.h"
#include "Controller.h"
#include "IDAPBC.h"
#include "rPBC.h"
#include "CMM.h"
#include "elisa3-lib.h"

int main(int argc, char **argv){

    // Initialise ROS
    ros::init(argc, argv, "elisa_3");
    
    ros::NodeHandle nh;
    std::string output;
    helpers::safelyRetrieve(nh, "/output", output);

    // Create a system and a controller
    std::unique_ptr<System> system = std::make_unique<Elisa3>();
    std::unique_ptr<Controller> controller;
    
    if(output == "z"){
         controller = std::make_unique<IDAPBC>(*(system->cmm->agent));
    }else{
         controller = std::make_unique<rPBC>(*(system->cmm->agent));
    }
    
    // Define loop rate
    ros::Rate loop_rate(system->cmm->agent->getSamplingRate());

    while(ros::ok()){
        if(system->dataReady()) {
            
            /// Retrieve the cooperative input
            Eigen::VectorXd tau_network = system->cmm->sample(controller->getOutput(*system));

            /// Compute the control input
            Eigen::VectorXd tau = controller->computeControl(*system, tau_network);

            /// Send the input to the system
            system->sendInput(tau);
        }

        // Check callbacks and sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}