/*

File: IDAPBC.cpp

Implements an IDAPBC controller for the panda robotic arm.

*/


#include "IDAPBC.h"

/* Possibly: Use a shared pointer where the system is shared between the agent_node and the controller*/
IDAPBC::IDAPBC(int l_dim, std::unique_ptr<System>& system) : 
	Controller(l_dim)
{
	logMsg("IDAPBC", "Initiating.. ", 2);

	Eigen::VectorXd Vs = Eigen::VectorXd::Zero(system->n);
	Eigen::VectorXd ts = Eigen::VectorXd::Zero(system->n);


	ros::NodeHandle n_private("~");
	for(int i = 0; i < system->n; i++){
		n_private.getParam("Vs/" + std::to_string(i), Vs[i]);
		n_private.getParam("ts/" + std::to_string(i), ts[i]);
	}

	setdVsdq(system, Vs, ts);

	logMsg("IDAPBC", "Done!", 2);

}

Eigen::VectorXd IDAPBC::computeControl(std::unique_ptr<System>& system, Eigen::VectorXd tau_c){
	
	// Initialise the control input
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(system->m);

	// Apply IDA-PBC
	tau = system->getdHdq() - 1.0*getKv(system)*system->state.dq - getdVsdq(system);


	// Add the cooperative input
	tau = tau + getPsi(system) * tau_c;// - psi*psi.transpose()*system->state.dq;

	return tau;
}


Eigen::VectorXd IDAPBC::getOutput(std::unique_ptr<System>& system){

	// Define the output
	Eigen::VectorXd r(l);

	// Set it to be z
	r = system->getEEPose();//+ getPsi(system).transpose()*system->state.dq;

	// Publish it for debugging
	this->publishZ(r);

	// Return the output
	return r;
}


bool IDAPBC::setdVsdq(std::unique_ptr<System>& system, Eigen::VectorXd new_dVs, Eigen::VectorXd new_theta_star){
	
	// Catch bad inputs
	if(new_dVs.size() != system->n || new_theta_star.size() != system->n){
		logMsg("IDAPBC", "dVs size does not match the system size\n Changes were not applied!\n", 1);

		return false;
	}

	// If inputs where correct apply them
	dVs = new_dVs;
	theta_star = new_theta_star;

	return true;
}

// Define the local gradient
Eigen::VectorXd IDAPBC::getdVsdq(std::unique_ptr<System>& system){
	// Return a linear gradient w.r.t. local coordinates
	return -dVs.cwiseProduct(theta_star - system->state.q);
}

// Define damping
Eigen::MatrixXd IDAPBC::getKv(std::unique_ptr<System>& system){
	return Eigen::MatrixXd::Identity(system->n, system->n);
}


// Define dzdq
Eigen::MatrixXd IDAPBC::getPsi(std::unique_ptr<System>& system){
	Eigen::MatrixXd Psi = Eigen::MatrixXd::Zero(system->n, l);
	Eigen::Matrix<double, 7, 1> q = system->state.q;
Psi(0,0) = 0.0825*sin(q(0))*sin(q(1))*sin(q(3)) - 0.316*sin(q(0))*sin(q(1)) - 0.0825*cos(q(0))*sin(q(2)) - 0.0825*cos(q(1))*cos(q(2))*sin(q(0)) + 0.0825*cos(q(0))*cos(q(3))*sin(q(2)) - 0.384*cos(q(3))*sin(q(0))*sin(q(1)) + 0.384*cos(q(0))*sin(q(2))*sin(q(3)) + 0.0825*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0)) - 0.088*cos(q(0))*cos(q(2))*cos(q(5))*sin(q(4)) + 0.384*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3)) - 0.088*cos(q(3))*sin(q(0))*sin(q(1))*sin(q(5)) + 0.088*cos(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2)) + 0.088*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3))*sin(q(5)) + 0.088*cos(q(1))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.088*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(3)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0));
Psi(0, 1) = 0.316*cos(q(0))*sin(q(1)) - 0.0825*sin(q(0))*sin(q(2)) + 0.384*sin(q(0))*sin(q(2))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(2)) + 0.384*cos(q(0))*cos(q(3))*sin(q(1)) - 0.0825*cos(q(0))*sin(q(1))*sin(q(3)) + 0.0825*cos(q(3))*sin(q(0))*sin(q(2)) - 0.0825*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)) - 0.384*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3)) + 0.088*cos(q(0))*cos(q(3))*sin(q(1))*sin(q(5)) - 0.088*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(4)) + 0.088*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4)) + 0.088*cos(q(0))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.088*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2)) + 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5));
Psi(0, 2) = 0;
Psi(1, 0) = 0.316*cos(q(0))*cos(q(1)) + 0.384*cos(q(0))*cos(q(1))*cos(q(3)) - 0.0825*cos(q(0))*cos(q(2))*sin(q(1)) - 0.0825*cos(q(0))*cos(q(1))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(2))*cos(q(3))*sin(q(1)) + 0.088*cos(q(0))*cos(q(1))*cos(q(3))*sin(q(5)) + 0.384*cos(q(0))*cos(q(2))*sin(q(1))*sin(q(3)) + 0.088*cos(q(0))*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(3)) + 0.088*cos(q(0))*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(0))*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.088*cos(q(0))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1));
Psi(1, 1) = 0.316*cos(q(1))*sin(q(0)) + 0.384*cos(q(1))*cos(q(3))*sin(q(0)) - 0.0825*cos(q(2))*sin(q(0))*sin(q(1)) - 0.0825*cos(q(1))*sin(q(0))*sin(q(3)) + 0.0825*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(1)) + 0.088*cos(q(1))*cos(q(3))*sin(q(0))*sin(q(5)) + 0.384*cos(q(2))*sin(q(0))*sin(q(1))*sin(q(3)) + 0.088*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(3)) + 0.088*cos(q(2))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.088*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1));
Psi(1, 2) = 0.0825*sin(q(1))*sin(q(3)) - 0.0825*cos(q(1))*cos(q(2)) - 0.384*cos(q(3))*sin(q(1)) - 0.316*sin(q(1)) + 0.0825*cos(q(1))*cos(q(2))*cos(q(3)) + 0.384*cos(q(1))*cos(q(2))*sin(q(3)) - 0.088*cos(q(3))*sin(q(1))*sin(q(5)) + 0.088*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5)) + 0.088*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4)) - 0.088*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5));
Psi(2, 0) = 0.0825*cos(q(2))*cos(q(3))*sin(q(0)) - 0.0825*cos(q(0))*cos(q(1))*sin(q(2)) - 0.0825*cos(q(2))*sin(q(0)) + 0.384*cos(q(2))*sin(q(0))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(3))*sin(q(2)) + 0.384*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(3)) + 0.088*cos(q(2))*sin(q(0))*sin(q(3))*sin(q(5)) + 0.088*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(4)) - 0.088*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0)) + 0.088*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2));
Psi(2, 1) = 0.0825*cos(q(0))*cos(q(2)) - 0.0825*cos(q(0))*cos(q(2))*cos(q(3)) - 0.384*cos(q(0))*cos(q(2))*sin(q(3)) - 0.0825*cos(q(1))*sin(q(0))*sin(q(2)) + 0.0825*cos(q(1))*cos(q(3))*sin(q(0))*sin(q(2)) - 0.088*cos(q(0))*cos(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(5))*sin(q(2))*sin(q(4)) + 0.384*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(3)) + 0.088*cos(q(0))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5)) - 0.088*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(4)) + 0.088*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2));
Psi(2, 2) = 0.0825*sin(q(1))*sin(q(2)) - 0.384*sin(q(1))*sin(q(2))*sin(q(3)) - 0.0825*cos(q(3))*sin(q(1))*sin(q(2)) + 0.088*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(4)) - 0.088*sin(q(1))*sin(q(2))*sin(q(3))*sin(q(5)) + 0.088*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(2));
Psi(3, 0) = 0.384*cos(q(3))*sin(q(0))*sin(q(2)) - 0.0825*cos(q(0))*cos(q(3))*sin(q(1)) - 0.384*cos(q(0))*sin(q(1))*sin(q(3)) - 0.0825*sin(q(0))*sin(q(2))*sin(q(3)) - 0.384*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3)) - 0.088*cos(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(3))*sin(q(0))*sin(q(2))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(5)) + 0.088*cos(q(0))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1)) + 0.088*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(3)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(3));
Psi(3, 1) = 0.0825*cos(q(0))*sin(q(2))*sin(q(3)) - 0.384*cos(q(0))*cos(q(3))*sin(q(2)) - 0.0825*cos(q(3))*sin(q(0))*sin(q(1)) - 0.384*sin(q(0))*sin(q(1))*sin(q(3)) - 0.384*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0)) + 0.0825*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3)) - 0.088*cos(q(0))*cos(q(3))*sin(q(2))*sin(q(5)) - 0.088*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(5)) + 0.088*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1)) - 0.088*cos(q(0))*cos(q(4))*cos(q(5))*sin(q(2))*sin(q(3)) - 0.088*cos(q(1))*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(3));
Psi(3, 2) = 0.384*cos(q(2))*cos(q(3))*sin(q(1)) - 0.384*cos(q(1))*sin(q(3)) - 0.0825*cos(q(1))*cos(q(3)) - 0.0825*cos(q(2))*sin(q(1))*sin(q(3)) - 0.088*cos(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5)) + 0.088*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(5)) + 0.088*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3));
Psi(4, 0) = 0.088*cos(q(3))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.088*cos(q(0))*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(2)) - 0.088*cos(q(0))*cos(q(5))*sin(q(1))*sin(q(3))*sin(q(4)) - 0.088*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(0)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(4));
Psi(4, 1) = 0.088*cos(q(0))*cos(q(2))*cos(q(4))*cos(q(5)) - 0.088*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2)) - 0.088*cos(q(0))*cos(q(3))*cos(q(5))*sin(q(2))*sin(q(4)) - 0.088*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(4)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(0))*sin(q(4));
Psi(4, 2) = 0.088*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(2)) - 0.088*cos(q(1))*cos(q(5))*sin(q(3))*sin(q(4)) + 0.088*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(1))*sin(q(4));
Psi(5, 0) = 0.088*cos(q(0))*cos(q(3))*cos(q(5))*sin(q(1)) + 0.088*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(3)) + 0.088*cos(q(2))*sin(q(0))*sin(q(4))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(3)) + 0.088*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) - 0.088*cos(q(0))*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(2))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(5));
Psi(5, 1) = 0.088*cos(q(3))*cos(q(5))*sin(q(0))*sin(q(1)) - 0.088*cos(q(0))*cos(q(5))*sin(q(2))*sin(q(3)) - 0.088*cos(q(0))*cos(q(2))*sin(q(4))*sin(q(5)) - 0.088*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(3)) - 0.088*cos(q(0))*cos(q(3))*cos(q(4))*sin(q(2))*sin(q(5)) + 0.088*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(4))*sin(q(5)) - 0.088*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(5));
Psi(5, 2) = 0.088*cos(q(1))*cos(q(3))*cos(q(5)) + 0.088*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.088*cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5)) - 0.088*sin(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) + 0.088*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(5));
	//std::cout << "Psi: \n" << Psi << "\n";
	return Psi;
}