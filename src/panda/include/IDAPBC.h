/*
File: IDAPBC.h

A standard IDA-PBC controller, implements the controller interface defined in Controller.h
*/

#ifndef IDAPBC_H
#define IDAPBC_H

#include "ros/ros.h"
#include <ros/console.h>
#include "Controller.h"
#include "CustomLog.h"

#include <sstream>

class IDAPBC : public Controller{

public:
	IDAPBC(int l_dim, System& system);

	Eigen::VectorXd computeControl(System& system, Eigen::VectorXd tau_c);

	// Get the agent output, possibly modified
	Eigen::VectorXd getOutput(System& system);

	// Set the local potential gradient as quadratic with specified gains
	bool setdVsdq(System& system, Eigen::VectorXd new_dVs, Eigen::VectorXd new_theta_star);


private:

	// Gains
	Eigen::VectorXd dVs, theta_star;

	/* Parameters necessary for IDA-PBC control */
	// Define the local gradient
	Eigen::VectorXd getdVsdq(System& system);

	// Define damping
	Eigen::MatrixXd getKv(System& system);

	// Define the input matrix
	Eigen::MatrixXd getPsi(System& system);

};


#endif