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
#include "CMM.h"

int main(int argc, char **argv){

    // Initialise ROS
    ros::init(argc, argv, "elisa_3");

    // Get a nodehandle
    ros::NodeHandle nh("/elisa3/");

    // Retrieve important parameters
    int id, address, sampling_rate;
    helpers::safelyRetrieve(nh, "ID", id);
    helpers::safelyRetrieve(nh, "address", address);
    helpers::safelyRetrieve(nh, "sampling_rate", sampling_rate, 100);

    double initial_delay;
    helpers::safelyRetrieve(nh, "initial_delay", initial_delay, 2.0);


    // Create a system and a controller
    std::unique_ptr<System> system = std::make_unique<Elisa3>(address, sampling_rate);
    std::unique_ptr<Controller> controller = std::make_unique<IDAPBC>(*system);

    // Create a Communication Management Module
    CMM cmm(id);

    // Define loop rate
    ros::Rate loop_rate(sampling_rate);

    // Wait for everything to startup
    ros::Duration(initial_delay).sleep();

    while(ros::ok()){

        /// Read sensors
        system->readSensors();

        /// Retrieve the cooperative input
        Eigen::VectorXd tau_network = cmm.sample(controller->getOutput(*system));

        /// Compute the control input
        Eigen::VectorXd tau = controller->computeControl(*system, tau_network);

        /// Send the input to the system
        system->sendInput(tau);

        // Check callbacks and sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}