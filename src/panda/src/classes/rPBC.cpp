/*

File: rPBC.cpp

Implements an rPBC controller for the panda robotic arm.

*/

#include "rPBC.h"

/* Possibly: Use a shared pointer where the system is shared between the agent_node and the controller*/
rPBC::rPBC(System& system) 
	: IDAPBC(system)
{
	logMsg("rPBC", "Initiating.. ", 2);

	/* Retrieve controller gains */
	ros::NodeHandle nh;
	helpers::safelyRetrieve(nh, "/lambda", lambda);


	logMsg("rPBC", "Done!", 2);
}

Eigen::VectorXd rPBC::computeControl(System& system, Eigen::VectorXd tau_c){
	
	// Initialise the control input
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(system.m);

	// Compensate gravity if necessary
	if(gravity_enabled){
		tau += system.dVdq();
	}

	// Doesn seem to exist in Eigen3.0
	//Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(system.Psi());
	//Eigen::MatrixXd pinvPsi = (system.Psi().pseudoInverse());
//.completeOrthogonalDecomposition()).pseudoInverse()

	Eigen::MatrixXd psi_pinv = rightPseudoInverse(system.Psi());

	Eigen::FullPivLU<Eigen::MatrixXd> psi_lu(system.Psi());
	Eigen::MatrixXd nullPsi = psi_lu.kernel();

	Eigen::VectorXd tau_c_hat = tau_c +
		system.Psi().transpose()*system.M()*getKv(system)*system.state.dq;

	tau += system.M()*(psi_pinv*tau_c_hat
					-lambda*system.state.dq-
					getKv(system)*system.state.dq);// - 
					//nullPsi.transpose()*nullPsi*getdVsdq(system));
	

	//logTmp(tau);
	return tau;
}


Eigen::VectorXd rPBC::getOutput(System& system){

	// Define the output
	Eigen::VectorXd r(l);

	// r = lambda*z + zdot
	r = lambda*system.state.z + system.Psi().transpose()*system.state.dq;

	// Publish it for debugging
	publishValue(z_pub, z_rate, system.state.z);

	// Return the output
	return r;
}

Eigen::MatrixXd rPBC::rightPseudoInverse(Eigen::MatrixXd A){
	return A*(A.transpose()*A).inverse();
}

