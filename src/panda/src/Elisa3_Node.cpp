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
#include "elisa3-lib.h"

int main(int argc, char **argv){

    // Initialise ROS
    ros::init(argc, argv, "elisa_3");

    // Get a nodehandle
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh("/elisa3/");

    // Retrieve important agent specific parameters
    int id, address, sampling_rate;
    helpers::safelyRetrieve(nh_private, "ID", id);
    helpers::safelyRetrieve(nh_private, "address", address);
//    Eigen::VectorXd init_state;
//    helpers::safelyRetrieveEigen(nh_private, "init_state", init_state, 3);
//    logTmp(init_state);

    double initial_delay;
    helpers::safelyRetrieve(nh, "sampling_rate", sampling_rate, 100);
    helpers::safelyRetrieve(nh, "initial_delay", initial_delay, 2.0);

    // Create a system and a controller
    std::unique_ptr<System> system = std::make_unique<Elisa3>(address, sampling_rate);
    std::unique_ptr<Controller> controller = std::make_unique<IDAPBC>(*(system->cmm->agent));

    // Create a Communication Management Module
    //CMM cmm("elisa_3");//id, system->agent->getSamplingRate());

    // Define loop rate
    ros::Rate loop_rate(sampling_rate);

    // Wait for everything to startup
    ros::Duration(initial_delay).sleep();
    int counter_yes = 0;
    int counter_no = 0;
    while(ros::ok()){
        if(system->dataReady()) {
            counter_yes++;
            /// Retrieve the cooperative input
            Eigen::VectorXd tau_network = system->cmm->sample(controller->getOutput(*system));

            /// Compute the control input
            Eigen::VectorXd tau = controller->computeControl(*system, tau_network);

            /// Send the input to the system
            system->sendInput(tau);
        }else{
            counter_no++;
            //logTmp(ros::Time::now());
        }

        // Check callbacks and sleep
        ros::spinOnce();
        loop_rate.sleep();
    }
    logTmp("y", counter_yes);
    logTmp("n", counter_no);
    return 0;

}