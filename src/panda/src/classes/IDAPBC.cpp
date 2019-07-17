/*

File: IDAPBC.cpp

Implements an IDAPBC controller for the panda robotic arm.

*/


#include "IDAPBC.h"

/* Possibly: Use a shared pointer where the system is shared between the agent_node and the controller*/
IDAPBC::IDAPBC(System& system)
{
	logMsg("IDAPBC", "Initiating.. ", 2);

	/* Retrieve controller gains */
	ros::NodeHandle nh;
	std::vector<double> Vs_gains_v, theta_star_v;

	helpers::safelyRetrieveArray(nh, "/controller/Vs_gains", Vs_gains_v, 7);
	helpers::safelyRetrieveArray(nh, "/controller/theta_star", theta_star_v, 7);
	helpers::safelyRetrieve(nh, "/l", l);

	Vs_gains = helpers::vectorToEigen(Vs_gains_v);
	theta_star = helpers::vectorToEigen(theta_star_v);

	logMsg("IDAPBC", "Done!", 2);

}

Eigen::VectorXd IDAPBC::computeControl(System& system, Eigen::VectorXd tau_c){
	
	// Initialise the control input
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(system.m);

	// Apply IDA-PBC (The robot autocompensates for gravity?)
	tau = -getdVsdq(system)- getKv(system)*system.state.dq; //0.1*system.dVdq();//  - getVs_gainsdq(system);


	// Add the cooperative input
	//tau = tau + getPsi(system) * tau_c;// - psi*psi.transpose()*system.state.dq;

	return tau;
}


Eigen::VectorXd IDAPBC::getOutput(System& system){

	// Define the output
	Eigen::VectorXd r(l);

	// Set it to be z
	r = system.state.z;//+ getPsi(system).transpose()*system.state.dq;

	// Publish it for debugging
	this->publishZ(r);

	// Return the output
	return r;
}

// Define the local gradient
Eigen::VectorXd IDAPBC::getdVsdq(System& system){

	// Return a linear gradient w.r.t. local coordinates
	return -Vs_gains.cwiseProduct(theta_star - system.state.q);
}

// Define damping
Eigen::MatrixXd IDAPBC::getKv(System& system){
	return 0.05*Eigen::MatrixXd::Identity(system.n, system.n);
}


// Define dzdq
Eigen::MatrixXd IDAPBC::getPsi(System& system){
	Eigen::MatrixXd Psi = Eigen::MatrixXd::Zero(system.n, l);
	Eigen::Matrix<double, 7, 1> q = system.state.q;
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