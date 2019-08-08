//
// Created by root on 07-08-19.
//

#include "Helpers.h"
#include "Elisa3.h"
#include "System.h"
#include "Controller.h"
#include "IDAPBC.h"
#include "CMM.h"


int main(int argc, char **argv){

    // Initialise ROS
    ros::init(argc, argv, "Elisa3");

    ros::NodeHandle nh = ros::NodeHandle("~");

    // Get a nodehandle
//    helpers::safelyRetrieve(nh, "/panda/ID", id);
//    helpers::safelyRetrieve(nh, "/l", l);
//    helpers::safelyRetrieve(nh, "/N_agents", N);
    int address, sampling_rate;
    helpers::safelyRetrieve(nh, "address", address, 3656);
    helpers::safelyRetrieve(nh, "sampling_rate", sampling_rate, 100);

    std::unique_ptr<System> system = std::make_unique<Elisa3>();
    std::unique_ptr<Controller> controller = std::make_unique<IDAPBC>(*system);

    CMM cmm(id);

    //Maybe also define the controller here-> Agent has a system and a controller

    ros::Rate loop_rate(sampling_rate);

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

    //delete system;
    //delete controller;
    // Not sure if necessary but still
    //delete edges;

    return 0;

}