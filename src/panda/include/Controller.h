/*

File: Controller.h

An interface for controllers using IDAPBC or rPBC control.

*/

#ifndef Controller_H
#define Controller_H

#include "ros/ros.h"

#include "System.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "std_msgs/Float64MultiArray.h"

#include <realtime_tools/realtime_publisher.h>
#include <franka_hw/trigger_rate.h>

#include "Helpers.h"

class Controller{

public:
	Controller();
	~Controller();

	int l;

	ros::NodeHandle nh;

	// Realtime publishing
	franka_hw::TriggerRate tau_rate{1.0};
	franka_hw::TriggerRate z_rate{1.0};
	realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> tau_pub, z_pub;

    // Get the agent output, possibly modified
    virtual Eigen::VectorXd getOutput(System& system) = 0;

    // Compute the input (POSSIBLY AT A ROBOTSTATE HERE)
    virtual Eigen::VectorXd computeControl(System& system, Eigen::VectorXd tau_c) = 0;

    void publishValue(realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>& pub,
            franka_hw::TriggerRate trigger_rate, const Eigen::VectorXd values);

private:


};


#endif