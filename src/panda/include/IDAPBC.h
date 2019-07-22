/*
File: IDAPBC.h

A standard IDA-PBC controller, implements the controller interface defined in Controller.h
*/

#ifndef IDAPBC_H
#define IDAPBC_H

#include "ros/ros.h"
#include "Controller.h"
#include "CustomLog.h"
#include "Helpers.h"

class IDAPBC : public Controller{

public:
	IDAPBC(System& system);

	Eigen::VectorXd computeControl(System& system, Eigen::VectorXd tau_c);

	// Get the agent output, possibly modified
	Eigen::VectorXd getOutput(System& system);

	/* Parameters necessary for IDA-PBC control */
	// Define the local gradient
	Eigen::VectorXd getdVsdq(System& system);

	// Define damping
	Eigen::MatrixXd getKv(System& system);

	// Gains
	Eigen::VectorXd Vs_gains, theta_star, limit_avoidance_gains, limits_avg;
	bool local_enabled, limit_avoidance_enabled, gravity_enabled,integral_enabled;
	double kv, ki;

	Eigen::VectorXd integral_state;


private:

	Eigen::MatrixXd rightPseudoInverse(Eigen::MatrixXd A);

};


#endif