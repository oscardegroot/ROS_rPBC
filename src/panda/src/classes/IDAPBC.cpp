
#include "IDAPBC.h"

/* Possibly: Use a shared pointer where the system is shared between the agent_node and the controller*/
IDAPBC::IDAPBC(int l_dim, System * system) : 
	Controller(l_dim)
{
	std::cout << "[IDAPBC] Initiating IDAPBC controller\n";
	setdVsdq(system, Eigen::VectorXd::Zero(system->n));
	std::cout << "[IDAPBC] IDAPBC controller is ready!\n";
}

Eigen::VectorXd IDAPBC::computeControl(System * system, Eigen::VectorXd tau_c){
	
	// Initialise the control input
	Eigen::VectorXd tau(system->m);


	// Fully actuated for now!
	tau = system->getdHdq()- getdVsdq(system) - getKv(system)*system->state.dq;

	// Add the cooperative input
	tau += getPsi(system) * tau_c;

	// return tau;
	return tau;
}

Eigen::VectorXd IDAPBC::getOutput(System * system){
	Eigen::VectorXd r(l);

	r << 1, 1;
	return r;

	//return Eigen::VectorXd::Random(this->l);
}


bool IDAPBC::setdVsdq(System * system, Eigen::VectorXd new_dVs){
	if(new_dVs.size() != system->n){
		std::cout << "[IDAPBC]: Warning dVs size does not match the system size\n" <<
					 "			!Changes were not applied!\n";

		return false;
	}

	dVs = new_dVs;
	return true;
}

// Define the local gradient
Eigen::VectorXd IDAPBC::getdVsdq(System * system){
	// Return a linear gradient w.r.t. local coordinates
	return dVs*dVs.transpose()*system->state.q;
}

// Define damping
Eigen::MatrixXd IDAPBC::getKv(System * system){
	return Eigen::MatrixXd::Identity(system->n, system->n);
}

Eigen::MatrixXd IDAPBC::getPsi(System * system){
	Eigen::MatrixXd Psi(system->n, l);
	
		// Demo for 3x2
	// Psi <<  1, 0,
	// 		0, 1,
	// 		std::cos(system->state.q(2)), std::sin(system->state.q(1));

	return Eigen::MatrixXd::Zero(system->n, l);
}