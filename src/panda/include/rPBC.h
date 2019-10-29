/*
File: rPBC.h

A standard IDA-PBC controller, implements the controller interface defined in Controller.h
*/

#ifndef rPBC_H
#define rPBC_H

#include "ros/ros.h"
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include "Controller.h"
#include "CustomLog.h"
#include "Helpers.h"
#include "IDAPBC.h"


class rPBC : public Controller{

public:
	rPBC(Agent& agent);

	Eigen::VectorXd computeControl(System& system, const Eigen::VectorXd& tau_c) override;

	// Get the agent output, possibly modified
	Eigen::VectorXd getOutput(System& system) override;

    Eigen::VectorXd dVsdq(System& system) override;

    Eigen::MatrixXd Kv(System& system) override;

    void publishAll(System& system) override;

	//Eigen::MatrixXd rightPseudoInverse(Eigen::MatrixXd A);


private:

	double lambda, gamma, kappa;
    
    Benchmarker benchmarker;
    
    Eigen::MatrixXd null_psi, pinv_psi, dnull_psi, null_pinv_psi;
    
    Eigen::MatrixXd previous_null_psi;
    
    Eigen::MatrixXd& pinvPsi(System& system);
    Eigen::MatrixXd& nullPsi(System& system);
    Eigen::MatrixXd& dnullPsi(System& system);
    Eigen::MatrixXd& nullPinvPsi(System& system);

    
    bool pinv_psi_updated, null_psi_updated, dnull_psi_updated, null_pinv_psi_updated;
    bool initial_run = true;
    bool has_local_freedom;

};


#endif