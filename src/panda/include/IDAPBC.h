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
// Agent should be given here
	IDAPBC(Agent& agent);

// Model here...
	Eigen::VectorXd computeControl(System& system, const Eigen::VectorXd& tau_c) override;

	// Get the agent output, possibly modified
	Eigen::VectorXd getOutput(System& system) override;

	/* Parameters necessary for IDA-PBC control */
	// Define the local gradient
	Eigen::VectorXd dVsdq(System& system) override;
    void publishAll(System& system) override;

	// Define damping
	//Eigen::MatrixXd Kv(System& system) override;

private:

    Benchmarker benchmark;
    //Eigen::MatrixXd rightPseudoInverse(const Eigen::MatrixXd& A) const;
    //Eigen::MatrixXd pinv_psi;


};



#endif