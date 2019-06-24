/*
File: Controller.h

Define a connection with another agent that uses the ST and WVM to passify communication
*/

#ifndef Controller_H
#define Controller_H

#include "ros/ros.h"

#include "System.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class Controller{

public:
	Controller(int l_dim);
	~Controller();

	int l;

	// Get the agent output, possibly modified
	virtual Eigen::VectorXd getOutput(std::unique_ptr<System>& system) = 0;

	// Compute the input (POSSIBLY AT A ROBOTSTATE HERE)
	virtual Eigen::VectorXd computeControl(std::unique_ptr<System>& system, Eigen::VectorXd tau_c) = 0;

private:




};


#endif