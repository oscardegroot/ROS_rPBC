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

	agent.retrieveParameter("controller/kq", kq, 1.0);
	agent.retrieveParameter("controller/kz", kz, 1.0);
//    agent.retrieveParameter("eta", eta, 1.0);
//    helpers::safelyRetrieve(nh, "/network/delay/max_delay", max_delay);
//    eta_startup = 100.0;
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

    // In the initial run, set initial matrices
//    if(!initial_run){
//        timer = helpers::SimpleTimer(3.0*max_delay);
//        initial_run = true;
//    }

    benchmark.start();
	// Initialise the control input
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(system.m);
    
    // Quick fix: set eta to high value while t < T
//    double cur_eta = eta;
//    if(!timer.finished()){
//
//        //cur_eta = eta_startup;
//        return tau;
//    }
    
    if(!system.isEnabled()){
        return tau;
    }
    
	// Compensate gravity if necessary
	if(gravity_enabled){
		tau += system.dVdq();
	}

    // Apply IDA-PBC (The actual panda autocompensates for gravity)
	tau += -dVsdq(system);
	tau += -kq * system.state.dq;
	
    // Add the cooperative input
	Eigen::MatrixXd pinv_psi;
	pinv_psi = helpers::pseudoInverse(system.Psi().transpose());
    //(1.0/cur_eta)*
    // Use IDA-PBC for fully actuated agents to find an input
	tau += pinv_psi * (tau_c - kz*system.Psi().transpose()*system.state.dq);
    
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
	publishAll(system);

	// Return the output
	return r;
}

void IDAPBC::publishAll(System& system){
    
    // Publish it for debugging
    publish("z", system.state.z);
    publish("z_dot", system.Psi().transpose()*system.state.dq);

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
