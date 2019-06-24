
#include "IDAPBC.h"

/* Possibly: Use a shared pointer where the system is shared between the agent_node and the controller*/
IDAPBC::IDAPBC(int l_dim, std::unique_ptr<System>& system) : 
	Controller(l_dim)
{
	std::cout << "[IDAPBC] Initiating IDAPBC controller\n";

	Eigen::VectorXd Vs = Eigen::VectorXd::Zero(system->n);
	Eigen::VectorXd new_theta_star = Eigen::VectorXd::Zero(system->n);

	Vs(1) = 1;
	// Vs(0) = 0.5;
	// Vs(2) = 0.5;
	// Vs(3) = 2;
	// Vs(4) = 0.5;
	// Vs(5) = 1;
	// Vs(6) = 0.5;

	new_theta_star(1) = -M_PI_2/2.0;

	setdVsdq(system, Vs, new_theta_star);
	std::cout << "[IDAPBC] IDAPBC controller is ready!\n";
}

Eigen::VectorXd IDAPBC::computeControl(std::unique_ptr<System>& system, Eigen::VectorXd tau_c){
	
	// Initialise the control input
	Eigen::VectorXd tau(system->m);

	// I can dampen the momenta for now...
	// Fully actuated for now!
	tau = system->getdHdq() - getdVsdq(system) -1*getKv(system)*system->state.dq;

	// Add the cooperative input
	//tau += getPsi(system) * tau_c;
	std::cout << "tau: \n" << tau << "\n";
	// return tau;
	return tau;
}

Eigen::VectorXd IDAPBC::getOutput(std::unique_ptr<System>& system){
	Eigen::VectorXd r(l);

	r << 1, 1;
	return r;

	//return Eigen::VectorXd::Random(this->l);
}


bool IDAPBC::setdVsdq(std::unique_ptr<System>& system, Eigen::VectorXd new_dVs, Eigen::VectorXd new_theta_star){
	if(new_dVs.size() != system->n || new_theta_star.size() != system->n){
		std::cout << "[IDAPBC]: Warning dVs size does not match the system size\n" <<
					 "			!Changes were not applied!\n";

		return false;
	}

	dVs = new_dVs;
	theta_star = new_theta_star;
	return true;
}

// Define the local gradient
Eigen::VectorXd IDAPBC::getdVsdq(std::unique_ptr<System>& system){
	// Return a linear gradient w.r.t. local coordinates
	return dVs*dVs.transpose()*(system->state.q- theta_star);
}

// Define damping
Eigen::MatrixXd IDAPBC::getKv(std::unique_ptr<System>& system){
	return Eigen::MatrixXd::Identity(system->n, system->n);
}

Eigen::MatrixXd IDAPBC::getPsi(std::unique_ptr<System>& system){
	Eigen::MatrixXd Psi(system->n, l);
	
		// Demo for 3x2
	// Psi <<  1, 0,
	// 		0, 1,
	// 		std::cos(system->state.q(2)), std::sin(system->state.q(1));

	return Eigen::MatrixXd::Zero(system->n, l);
}