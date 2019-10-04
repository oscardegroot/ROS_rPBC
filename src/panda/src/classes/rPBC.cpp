/*

File: rPBC.cpp

Implements an rPBC controller for the panda robotic arm.

*/

#include "rPBC.h"

/* Possibly: Use a shared pointer where the system is shared between the agent_node and the controller*/
rPBC::rPBC(Agent& agent) 
	: Controller(agent)
{
	logMsg("rPBC", "Initiating.. ", 2);

	/* Retrieve controller gains */
	ros::NodeHandle nh;
	helpers::safelyRetrieve(nh, "/lambda", lambda);
	helpers::safelyRetrieve(nh, "/gamma", gamma);

	logMsg("rPBC", "Done!", 2);
}

Eigen::VectorXd rPBC::computeControl(System& system, const Eigen::VectorXd& tau_c){
	
    ScopeTimer timer("rPBC Control"); // ~ 70 - 140 us!
    
	// Initialise the control input
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(system.m);
    
	// Compensate gravity if necessary
	if(gravity_enabled){
		tau += system.dVdq(); // maybe dMdq here as well...
        tau += 0.5*system.dM()*system.state.dq;// -> should be actual dMdq...
	}

	Eigen::MatrixXd pinv_psi;
    
    // Calculate pseudo inverse and null space
	pinv_psi = helpers::pseudoInverse(system.Psi().transpose());

//    // Null space calculations
//	Eigen::FullPivLU<Eigen::MatrixXd> psi_lu(system.Psi());
//	Eigen::MatrixXd nullPsi = psi_lu.kernel();
    
    Eigen::MatrixXd Kz = (lambda + gamma)*system.Psi().transpose() + system.dPsi().transpose() + 
                        system.Psi().transpose()*system.dMinv()*system.M(); // using inv(M)' = -M_dot * Minv * Mdot

    Eigen::VectorXd tau_hat = tau_c - Kz*system.state.dq + system.Psi().transpose()*Kv(system)*system.state.dq;
    
    // Use the rPBC control law to control the agent
	tau += system.M()*(pinv_psi*tau_hat - Kv(system)*system.state.dq);

    publishValue(tau_pub, tau_rate, tau);
//    logTmp("M", system.M());
//    logTmp("dM", system.dM());
//    logTmp("dMinv", system.dMinv());
//    logTmp("Psi", system.Psi());
//    logTmp("dPsi", system.dPsi());

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

// Define the local gradient
Eigen::VectorXd rPBC::dVsdq(System& system){

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

//Eigen::MatrixXd rPBC::rightPseudoInverse(Eigen::MatrixXd A){
//	return A*(A.transpose()*A).inverse();
//}

Eigen::MatrixXd rPBC::Kv(System& system)
{
    //return Eigen::MatrixXd::Zero(system.n, system.n);
    return system.dMinv()*system.M() + kq*Eigen::MatrixXd::Identity(system.n, system.n);// + system.dPsi()*helpers::pseudoInverse(system.Psi());
}
