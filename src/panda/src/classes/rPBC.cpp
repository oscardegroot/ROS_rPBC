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
    
    /** @todo Findout which parts are necessary (this is copypasta from idapbc */
    agent.retrieveParameter("controller/kq", kq, 1.0);
	agent.retrieveParameter("controller/kz", kz, 1.0);

	agent.retrieveParameter("controller/gravity_compensation/enabled", gravity_enabled, false);

	agent.retrieveParameter("controller/local_potential/enabled", local_enabled, false);

	if(local_enabled){
	    agent.retrieveEigen("controller/local_potential/gains", Vs_gains, 7);
	    agent.retrieveEigen("controller/local_potential/goals", theta_star, 7);
	}

	agent.retrieveParameter("controller/joint_limit_avoidance/enabled", limit_avoidance_enabled, false);

	if(limit_avoidance_enabled) {
        Eigen::VectorXd limits_min_array, limits_max_array;
        agent.retrieveEigen("controller/joint_limit_avoidance/gains", limit_avoidance_gains, 7);

        // Calculate the mid point of the joints
        agent.retrieveEigen("controller/joint_limit_avoidance/limits_min", limits_min_array, 7);
        agent.retrieveEigen("controller/joint_limit_avoidance/limits_max", limits_max_array, 7);

        //Convert to eigen
        limits_avg = 0.5 * (limits_min_array + limits_max_array);
    }
    /* Misses almost all parameters... classic */
    benchmarker = Benchmarker("rPBC", "rPBC Control");

	logMsg("rPBC", "Done!", 2);
}

Eigen::VectorXd rPBC::computeControl(System& system, const Eigen::VectorXd& tau_c){
	
    benchmarker.start();// ~ 70 - 140 us!

	// Initialise the control input
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(system.m);
    
	// Compensate gravity if necessary
	if(gravity_enabled){
		tau += system.dVdq(); // maybe dMdq here as well...
        //tau += system.dTdq();
        //tau += 0.5*system.dM()*system.state.dq;// -> should be actual dMdq...
	}

    /**@todo add dTdq() */

    // Calculate pseudo inverse and null space
    Eigen::MatrixXd pinv_psi;
	pinv_psi = helpers::pseudoInverse(system.Psi().transpose());

    // Null space calculations
	Eigen::FullPivLU<Eigen::MatrixXd> psi_lu(system.Psi().transpose());
	Eigen::MatrixXd nullPsi = psi_lu.kernel().transpose();
    
    /* We can simplify two terms with dMinv() * M. (There are three: in Kz, in Kv local and in Kv coop, only Kv local stays).*/
    Eigen::MatrixXd Kz = (lambda + gamma)*system.Psi().transpose() + system.dPsi().transpose();// + 
                        //system.Psi().transpose()*system.dMinv()*system.M(); // using inv(M)' = -M_dot * Minv * Mdot

    Eigen::VectorXd tau_hat = tau_c - Kz*system.state.dq + kq*system.Psi().transpose()*system.state.dq;// + system.Psi().transpose()*Kv(system)*system.state.dq;
    
    // Use the rPBC control law to control the agent
	tau += system.M()*(pinv_psi*tau_hat - (system.dM() + kq*Eigen::MatrixXd::Identity(system.n, system.n))*system.state.dq);

    // Local objectives
    tau -= nullPsi.transpose()*nullPsi*dVsdq(system);
    benchmarker.end();
    //publishValue(tau_pub, tau_rate, tau);

	return tau;
}

Eigen::VectorXd rPBC::getOutput(System& system){

	// r = lambda*z + zdot
	Eigen::VectorXd r(l);
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

Eigen::MatrixXd rPBC::Kv(System& system)
{
    //return Eigen::MatrixXd::Zero(system.n, system.n);
    return kq*Eigen::MatrixXd::Identity(system.n, system.n) + system.dMinv()*system.M();// + system.dPsi()*helpers::pseudoInverse(system.Psi());
}
