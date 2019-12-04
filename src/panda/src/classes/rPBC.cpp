/*

File: rPBC.cpp

Implements an rPBC controller for the panda robotic arm.

*/

#include "rPBC.h"

/* Possibly: Use a shared pointer where the system is shared between the agent_node and the controller*/
rPBC::rPBC(Agent& agent) 
	: Controller(agent)
{
    PROFILE_SCOPE("rPBC Init");
	logMsg("rPBC", "Initiating.. ", 2);
    null_psi_updated = false;
    pinv_psi_updated = false;
    dnull_psi_updated = false;
    null_pinv_psi_updated = false;
    
	/* Retrieve controller gains */
	ros::NodeHandle nh;
	helpers::safelyRetrieve(nh, "/lambda", lambda);
	helpers::safelyRetrieve(nh, "/gamma", gamma);
    helpers::safelyRetrieve(nh, "/kappa", kappa);

    agent.retrieveParameter("eta", eta, 1.0);

    gamma *= eta;
    eta_startup = 100.0;

    helpers::safelyRetrieve(nh, "/network/delay/max_delay", max_delay, 0.0);


	agent.retrieveParameter("controller/gravity_compensation/enabled", gravity_enabled, false);
	agent.retrieveParameter("controller/local_potential/enabled", local_enabled, false);

	if(local_enabled){
	    agent.retrieveEigen("controller/local_potential/gains", Vs_gains, 7);
	    agent.retrieveEigen("controller/local_potential/goals", theta_star, 7);
	}

	agent.retrieveParameter("controller/joint_limit_avoidance/enabled", limit_avoidance_enabled, false);

	if(limit_avoidance_enabled) {

        agent.retrieveEigen("controller/joint_limit_avoidance/gains", limit_avoidance_gains, 7);

        // Calculate the mid point of the joints
        agent.retrieveEigen("controller/joint_limit_avoidance/limits_min", limits_min, 7);
        agent.retrieveEigen("controller/joint_limit_avoidance/limits_max", limits_max, 7);
        limits_avg = 0.5 * (limits_min + limits_max);
    }

    benchmarker = Benchmarker("rPBC", "rPBC Control");
	logMsg("rPBC", "Done!", 2);
}

Eigen::VectorXd rPBC::computeControl(System& system, const Eigen::VectorXd& tau_c){
	
    PROFILE_FUNCTION();
    
    // In the initial run, set initial matrices
    if(initial_run){
        timer = helpers::SimpleTimer(3.0*max_delay);

        has_local_freedom = system.s > 0;
        
        // = (pinv(psi^T)) = n x l
        pinv_psi = Eigen::MatrixXd::Zero(system.n, system.n - system.s);
        
        // = psi^{null} *psi = 0 -> psi^null in (n - l) x n
        null_psi = Eigen::MatrixXd::Zero(system.s, system.n);
        
        if(has_local_freedom){
            dnull_psi = Eigen::MatrixXd::Zero(system.s, system.n);
            previous_null_psi = nullPsi(system);
        }
        initial_run = false;
    }
    
    benchmarker.start();// ~ 20 us!

	// Initialise the control input
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(system.m);

    double cur_eta = eta;

    // Quick fix: set eta to high value while t < T
    if(!timer.finished()){

        cur_eta = eta_startup;
        return tau;
    }

    
    if(!system.isEnabled()){
        return tau;
    }    
    
	// Compensate gravity if necessary
	if(gravity_enabled){
		tau += system.dVdq();
	}
    
    
    tau += system.C() - system.dM()*system.state.dq; // C includes the product with qdot

    
    Eigen::MatrixXd Kz = system.Psi().transpose()*((lambda + gamma) * Eigen::MatrixXd::Identity(system.n, system.n) -
                                                    system.M().inverse()*system.dM()) + system.dPsi().transpose();
    
    Eigen::VectorXd tau_hat = (1.0/cur_eta) * tau_c - Kz*system.state.dq;
    tau += system.M()*pinvPsi(system)*tau_hat;
    
    // If no local freedom, these computations are void...
    if(has_local_freedom){
        Eigen::MatrixXd Kv_mat = nullPsi(system) * (-system.M().inverse()*system.dM() + kappa*Eigen::MatrixXd::Identity(system.n, system.n)) + 
                                dnullPsi(system);
        tau -= system.M()*helpers::pseudoInverse(nullPsi(system))*(Kv_mat*system.state.dq
                                                            + helpers::pseudoInverse(nullPsi(system)).transpose()*dVsdq(system));
    }

    null_psi_updated = false;
    pinv_psi_updated = false;
    
    benchmarker.end();

	return tau;
}

Eigen::VectorXd rPBC::getOutput(System& system){

    PROFILE_FUNCTION();

	Eigen::VectorXd r(l);
	r = lambda*system.state.z + system.Psi().transpose()*system.state.dq;
    
    publishAll(system);
    
	// Return the output
	return r;
}

void rPBC::publishAll(System& system){
    
    PROFILE_FUNCTION();
    
    // Publish it for debugging
    publish("z", system.state.z);
    publish("z_dot", system.Psi().transpose()*system.state.dq);
    publish("theta_dot", nullPsi(system)*system.state.dq);

}

// Define the local gradient
Eigen::VectorXd rPBC::dVsdq(System& system){
    PROFILE_FUNCTION();
	Eigen::VectorXd dVsdq = Eigen::VectorXd::Zero(system.n);

	if(local_enabled){
		dVsdq -= Vs_gains.cwiseProduct(theta_star - system.state.q);
	}

	if(limit_avoidance_enabled){

		dVsdq -= limit_avoidance_gains.cwiseProduct(limits_avg - system.state.q);
	}
    
	return dVsdq;
}

Eigen::MatrixXd rPBC::Kv(System& system)
{
    PROFILE_FUNCTION();

    /** @note Minv_dot * m cancels! */
    Eigen::MatrixXd result = kappa*Eigen::MatrixXd::Identity(system.n, system.n);
    if(true && has_local_freedom){
        result += helpers::pseudoInverse(nullPsi(system)) * dnullPsi(system);
    }
    
    return result;
}

Eigen::MatrixXd& rPBC::pinvPsi(System& system)
{ 

    if(!pinv_psi_updated){
        PROFILE_FUNCTION();
        pinv_psi = helpers::pseudoInverse(system.Psi().transpose());
        pinv_psi_updated = true;
    }
    
    return pinv_psi;
}

Eigen::MatrixXd& rPBC::nullPsi(System& system)
{ 

    if(!null_psi_updated){
        PROFILE_FUNCTION();

        Eigen::FullPivLU<Eigen::MatrixXd> psi_lu(system.Psi().transpose());
        null_psi = psi_lu.kernel().transpose();
        null_psi_updated = true;
    }
    
    return null_psi;
}

Eigen::MatrixXd& rPBC::dnullPsi(System& system)
{

    if(!dnull_psi_updated){
        PROFILE_FUNCTION();

        Eigen::MatrixXd dnull_psi_input = system.approximateDerivative(nullPsi(system), previous_null_psi);
        helpers::lowpassFilter(dnull_psi, dnull_psi_input, 0.95);
        dnull_psi_updated = true;
    }
    //logTmp("dm", dm);

    return dnull_psi;
}