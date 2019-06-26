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
	Controller(int l_dim);
	~Controller();

	int l;

	ros::Publisher z_pub;	
	ros::NodeHandle nh;

	// Get the agent output, possibly modified
	virtual Eigen::VectorXd getOutput(std::unique_ptr<System>& system) = 0;

	// Compute the input (POSSIBLY AT A ROBOTSTATE HERE)
	virtual Eigen::VectorXd computeControl(std::unique_ptr<System>& system, Eigen::VectorXd tau_c) = 0;
	
	void publishZ(Eigen::VectorXd z);



private:



};


#endif