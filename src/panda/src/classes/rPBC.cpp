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

    // Retrieve system matrices
    system.Psi(psi);
    system.M(m);
    system.dVdq(dvdq);
    
	// Compensate gravity if necessary
	if(gravity_enabled){
		tau += dvdq;
	}

	// Doesn seem to exist in Eigen3.0
	//Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(system.Psi());
	//Eigen::MatrixXd pinvPsi = (system.Psi().pseudoInverse());
//.completeOrthogonalDecomposition()).pseudoInverse()

	Eigen::MatrixXd pinv_psi;
    
    // Calculate pseudo inverse and null space
	pinv_psi = pseudoInverse(psi.transpose());

	Eigen::FullPivLU<Eigen::MatrixXd> psi_lu(psi);
	Eigen::MatrixXd nullPsi = psi_lu.kernel();

	Eigen::VectorXd tau_c_hat = tau_c + lambda*psi.transpose()*system.state.dq;

	tau += m*pinv_psi*(tau_c_hat);// - 
					//nullPsi.transpose()*nullPsi*getdVsdq(system));
	//logTmp(tau);
	return tau;
}


Eigen::VectorXd rPBC::getOutput(System& system){

	// Define the output
	Eigen::VectorXd r(l);
    
    system.Psi(psi);

	// r = lambda*z + zdot
	r = lambda*system.state.z + psi.transpose()*system.state.dq;

	// Publish it for debugging
	publishValue(z_pub, z_rate, system.state.z);

	// Return the output
	return r;
}

Eigen::MatrixXd rPBC::rightPseudoInverse(Eigen::MatrixXd A){
	return A*(A.transpose()*A).inverse();
}

