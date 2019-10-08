/*

File: IDAPBC.cpp

Implements an IDAPBC controller for the panda robotic arm.

*/


#include "IDAPBC.h"

/* Possibly: Use a shared pointer where the system is shared between the agent_node and the controller*/
IDAPBC::IDAPBC(Agent& agent)
    : Controller(agent)
{
	logMsg("IDAPBC", "Initiating.. ", 2);

	/* Retrieve controller gains */
	ros::NodeHandle nh;

    /** @todo Replace all this for the agent.retrieveParameter implementation */
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
    
    benchmark = Benchmarker("IDA-PBC", "idapbc_control");

	logMsg("IDAPBC", "Done!", 2);

}

Eigen::VectorXd IDAPBC::computeControl(System& system, const Eigen::VectorXd& tau_c){
    //ScopeTimer timer("IDAPBC Control"); // ~25 us!
    
    benchmark.start();
	// Initialise the control input
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(system.m);

	// Compensate gravity if necessary
	if(gravity_enabled){
		tau += system.dVdq();
	}

    // Apply IDA-PBC (The actual panda autocompensates for gravity)
	tau += -dVsdq(system);
	tau += - kq * system.state.dq;
	
    // Add the cooperative input
	Eigen::MatrixXd pinv_psi;
	pinv_psi = helpers::pseudoInverse(system.Psi().transpose());

    // Use IDA-PBC for fully actuated agents to find an input
	tau += pinv_psi * (tau_c - kz*system.Psi().transpose()*system.state.dq);
    
    // Publish the input for debug purposes
	publishValue(tau_pub, tau_rate, tau);
    
    benchmark.end();
    // Return the input
	return tau;
}



Eigen::VectorXd IDAPBC::getOutput(System& system){

	// Define the output
	Eigen::VectorXd r(l);

	// Set it to be z (the selected values)
	r = system.state.z;

	// Publish it for debugging
	publishValue(z_pub, z_rate, r);

	// Return the output
	return r;
}

// Define the local gradient
Eigen::VectorXd IDAPBC::dVsdq(System& system){

	Eigen::VectorXd dVsdq = Eigen::VectorXd::Zero(system.n);

	if(local_enabled){
		dVsdq -= Vs_gains.cwiseProduct(theta_star - system.state.q);
	}

	if(limit_avoidance_enabled){
		dVsdq -= limit_avoidance_gains.cwiseProduct(limits_avg - system.state.q);
	}

	return dVsdq;
}
