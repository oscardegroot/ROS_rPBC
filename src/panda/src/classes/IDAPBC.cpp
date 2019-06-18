
#include "IDAPBC.h"


Eigen::VectorXd IDAPBC::computeControl(VectorXd q, VectorXd dq){
	
	// Initialise the control input
	Eigen::VectorXd tau(q.columns());

	tau = -getdVsdq(q);

	return tau;
}

Eigen::VectorXd IDAPBC::getdVsdq(VectorXd q){
	Eigen::VectorXd dVsdq = Eigen::VectorXd::Zero(q.columns());

	return dVsdq;
}
