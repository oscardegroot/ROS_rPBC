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

class Controller{

public:
	Controller();
	~Controller();

	int l;

	ros::Publisher z_pub, tau_pub;	
	ros::NodeHandle nh;

	// Get the agent output, possibly modified
	virtual Eigen::VectorXd getOutput(System& system) = 0;

	// Compute the input (POSSIBLY AT A ROBOTSTATE HERE)
	virtual Eigen::VectorXd computeControl(System& system, Eigen::VectorXd tau_c) = 0;
	
	void publishZ(Eigen::VectorXd z);
	void publishTau(Eigen::VectorXd tau);



private:



};


#endif