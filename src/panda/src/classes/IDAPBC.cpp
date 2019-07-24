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
	std::vector<double> limit_avoidance_gains_v, limits_min_array, limits_max_array;

	helpers::safelyRetrieve(nh, "/controller/kq", kq, 1.0);
	helpers::safelyRetrieve(nh, "/controller/kz", kz, 1.0);

	//helpers::safelyRetrieve(nh, "/controller/integral/ki", ki, 1.0);
	//helpers::safelyRetrieve(nh, "/controller/integral/enabled", integral_enabled, false);

	helpers::safelyRetrieve(nh, "/controller/gravity_compensation/enabled", gravity_enabled, false);

	helpers::safelyRetrieve(nh, "/controller/local_potential/enabled", local_enabled, false);
	helpers::safelyRetrieveArray(nh, "/controller/local_potential/gains", Vs_gains_v, 7);
	helpers::safelyRetrieveArray(nh, "/controller/local_potential/goals", theta_star_v, 7);
	
	helpers::safelyRetrieve(nh, "/controller/joint_limit_avoidance/enabled", limit_avoidance_enabled, false);
	helpers::safelyRetrieveArray(nh, "/controller/joint_limit_avoidance/gains", limit_avoidance_gains_v, 7);

	// Calculate the mid point of the joints
	helpers::safelyRetrieveArray(nh, "/controller/joint_limit_avoidance/limits_min", limits_min_array, 7);
	helpers::safelyRetrieveArray(nh, "/controller/joint_limit_avoidance/limits_max", limits_max_array, 7);
	
	//Convert to eigen
	limits_avg = 0.5*(helpers::vectorToEigen(limits_min_array) +
						helpers::vectorToEigen(limits_max_array));

	helpers::safelyRetrieve(nh, "/l", l);

	Vs_gains = helpers::vectorToEigen(Vs_gains_v);
	theta_star = helpers::vectorToEigen(theta_star_v);
	limit_avoidance_gains = helpers::vectorToEigen(limit_avoidance_gains_v);

	integral_state = Eigen::VectorXd::Zero(system.m);

	logMsg("IDAPBC", "Done!", 2);

}

Eigen::VectorXd IDAPBC::computeControl(System& system, Eigen::VectorXd tau_c){
	
	// Initialise the control input
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(system.m);

	// Compensate gravity if necessary
	if(gravity_enabled){
		tau += system.dVdq();
	}

	// Apply IDA-PBC (The robot autocompensates for gravity?)
	tau += -getdVsdq(system); //- getKv(system)*system.state.dq; //0.1*system.dVdq();//  - getVs_gainsdq(system);
	tau += - kq * system.state.dq;
	// Add the cooperative input
	Eigen::MatrixXd psi(system.Psi().block(0, 0, 7, 3));
	Eigen::MatrixXd pinv_psi;
	pinv_psi = pseudoInverse(psi.transpose());
	//logTmp(pinv_psi);
	//pseudoInverse(psi.transpose(), pinv_psi, 0.01);
	tau += pinv_psi * (tau_c- kz*psi.transpose()*system.state.dq);//- kz*psi.transpose()*system.state.dq);
	// tau += psi * (tau_c - kz*psi.transpose()*system.state.dq);// - psi*psi.transpose()*system.state.dq;
	publishTau(tau);
	//logTmp(tau);
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

	Eigen::VectorXd dVsdq = Eigen::VectorXd::Zero(system.n);

	if(local_enabled){
		dVsdq -= Vs_gains.cwiseProduct(theta_star - system.state.q);
	}

	if(limit_avoidance_enabled){
		dVsdq -= limit_avoidance_gains.cwiseProduct(limits_avg - system.state.q);
	}

	// Return a linear gradient w.r.t. local coordinates
	return dVsdq;
}

// Define damping (Not correct atm since kq / kz differ)
Eigen::MatrixXd IDAPBC::getKv(System& system){
	return Eigen::MatrixXd::Identity(system.n, system.n);
}

Eigen::MatrixXd IDAPBC::rightPseudoInverse(Eigen::MatrixXd A){
	
	return A*(A.transpose()*A).inverse();
}
