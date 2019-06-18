/*
File: Controller.h

Define a connection with another agent that uses the ST and WVM to passify communication
*/

#ifndef Controller_H
#define Controller_H

#include "ros/ros.h"

#include "Agent.h"

#include <vector>
#include <string>
#include <sstream>

class Controller{

public:
	Controller();
	~Controller();

	// Get the agent output, possibly modified
	virtual Eigen::VectorXd getOutput(VectorXd z_i, VectorXd q_i);

	// Compute the input (POSSIBLY AT A ROBOTSTATE HERE)
	virtual Eigen::VectorXd computeControl(VectorXd q_i, VectorXd dq_i) = 0;

private:




};


#endif