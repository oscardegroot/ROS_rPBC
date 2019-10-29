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
    null_psi_updated = false;
    pinv_psi_updated = false;
    dnull_psi_updated = false;
    null_pinv_psi_updated = false;
    
	/* Retrieve controller gains */
	ros::NodeHandle nh;
	helpers::safelyRetrieve(nh, "/lambda", lambda);
	helpers::safelyRetrieve(nh, "/gamma", gamma);
    helpers::safelyRetrieve(nh, "/kappa", kappa);

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

    /* Misses almost all parameters... classic */
    benchmarker = Benchmarker("rPBC", "rPBC Control");

	logMsg("rPBC", "Done!", 2);
}

Eigen::VectorXd rPBC::computeControl(System& system, const Eigen::VectorXd& tau_c){
	
    // In the initial run, set initial matrices
    if(initial_run){
        has_local_freedom = system.s > 0;
        
        // = (pinv(psi^T)) = n x l
        pinv_psi = Eigen::MatrixXd::Zero(system.n, system.n - system.s);
        
        // = psi^{null} *psi = 0 -> psi^null in (n - l) x n
        null_psi = Eigen::MatrixXd::Zero(system.s, system.n);
        
        if(has_local_freedom){
            dnull_psi = Eigen::MatrixXd::Zero(system.s, system.n);
            previous_null_psi = nullPsi(system);
        }
        logTmp("matrices initialised");
        initial_run = false;
    }
    
    benchmarker.start();// ~ 20 us!

	// Initialise the control input
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(system.m);
    
    if(!system.isEnabled()){
        return tau;
    }    
    
	// Compensate gravity if necessary
	if(gravity_enabled){
		tau += system.dVdq(); // maybe dMdq here as well...
        //tau += system.dTdq();
        //tau += 0.5*system.dM()*system.state.dq;// -> should be actual dMdq...
	}
    
    /* We can simplify two terms with dMinv() * M. (There are three: in Kz, in Kv local and in Kv coop, only Kv local stays).*/
    Eigen::MatrixXd Kz = (lambda + gamma)*system.Psi().transpose() + system.dPsi().transpose();// + 
                        //system.Psi().transpose()*system.dMinv()*system.M(); // using inv(M)' = -M_dot * Minv * Mdot

    Eigen::VectorXd tau_hat = tau_c - Kz*system.state.dq + 1.0*system.Psi().transpose()*system.state.dq;// + system.Psi().transpose()*Kv(system)*system.state.dq;
    
    // Use the rPBC control law to control the agent
	tau += system.M()*(pinvPsi(system)*tau_hat - (system.dM() + 1.0*Eigen::MatrixXd::Identity(system.n, system.n))*system.state.dq);

    // Local objectives
    tau -= nullPsi(system).transpose()*nullPsi(system)*dVsdq(system);
    
    /**@todo add dTdq() */
    /* We can simplify two terms with dMinv() * M. (There are three: in Kz, in Kv local and in Kv coop, only Kv local stays).*/
    //Eigen::MatrixXd Kz = (lambda + gamma)*system.Psi().transpose() + system.dPsi().transpose();// + 
                        //system.Psi().transpose()*system.dMinv()*system.M(); // using inv(M)' = -M_dot * Minv * Mdot

    //Eigen::VectorXd tau_hat = tau_c - Kz*system.state.dq + system.Psi().transpose()*Kv(system)*system.state.dq;// + system.Psi().transpose()*Kv(system)*system.state.dq;
//    Eigen::VectorXd tau_hat = tau_c - ((lambda+gamma-kappa)*system.Psi().transpose() + system.dPsi().transpose())*system.state.dq;
//    
//    // Use the rPBC control law to control the agent // (system.M()*system.dM() should be just system.dM()... But this works better?
//	tau += system.M()*pinvPsi(system)*tau_hat - (-system.dM() + system.M()*Kv(system))*system.state.dq;
    // Local
    /** @todo Without M behaviour significantly approves, but possible I just need to compensate for the gain of the input matrix.*/
//    logTmp("Norm of pinv_null_psi", (helpers::pseudoInverse(nullPsi(system)) * helpers::pseudoInverse(nullPsi(system)).transpose()).norm());
//    logTmp("Norm of old method", (nullPsi(system).transpose()*nullPsi(system)).norm());
    

    // Local objectives
//    if(has_local_freedom){
//        //tau -= system.M()*(helpers::pseudoInverse(nullPsi(system)) * helpers::pseudoInverse(nullPsi(system)).transpose()) * dVsdq(system); // seems to work
//
//        tau -= nullPsi(system).transpose()*nullPsi(system)*dVsdq(system);
//        dnull_psi_updated = false;
//        null_pinv_psi_updated = false;
//        previous_null_psi = nullPsi(system);
//    }
    //logTmp("tau: ", tau);
    null_psi_updated = false;
    pinv_psi_updated = false;
    
    benchmarker.end();
    //publish("tau", tau);

	return tau;
}

Eigen::VectorXd rPBC::getOutput(System& system){

	// r = lambda*z + zdot
	Eigen::VectorXd r(l);
	r = lambda*system.state.z + system.Psi().transpose()*system.state.dq;
    
    publishAll(system);
    
	// Return the output
	return r;
}

void rPBC::publishAll(System& system){
    
    // Publish it for debugging
    publish("z", system.state.z);
    publish("z_dot", system.Psi().transpose()*system.state.dq);
    publish("theta_dot", nullPsi(system)*system.state.dq);

}

// Define the local gradient
Eigen::VectorXd rPBC::dVsdq(System& system){

	Eigen::VectorXd dVsdq = Eigen::VectorXd::Zero(system.n);

	if(local_enabled){
		dVsdq -= Vs_gains.cwiseProduct(theta_star - system.state.q);
	}

    // Simple but works always...
	if(limit_avoidance_enabled){

		dVsdq -= limit_avoidance_gains.cwiseProduct(limits_avg - system.state.q);
	}


// Controls towards the limit_buffer point and keeps it there (unintended behaviour)
    // Activation based
//	if(limit_avoidance_enabled){
//        double limit_buffer = 0.5;
//        
//        for(size_t i = 0; i < system.n; i++){
//                      
//            double link_dif = limits_max(i) - limit_buffer - system.state.q(i);
//            if(link_dif < 0.0){
//                //logTmp("Limits are close!");
//                //logTmp(limit_avoidance_gains(i) * (link_dif));
//                dVsdq(i) -= limit_avoidance_gains(i) * (link_dif);
//            }else{
//
//                // if the state is smaller than the minimum plus the buffer (link_dif > 0)
//                link_dif = (limits_min(i) + limit_buffer) - system.state.q(i);
//                if(link_dif > 0.0){
//                    logTmp("Limits are close! (min)");
//
//                    logTmp(limit_avoidance_gains(i) * (link_dif));
//                    
//                    // Then control it closer (i.e., -link_dif?)
//                    dVsdq(i) += limit_avoidance_gains(i) * (link_dif);
//                }
//            }
//        }
//		//dVsdq -= limit_avoidance_gains.cwiseProduct(limits_avg - system.state.q);
//	}
// Does not seem to work (maybe purely because it conflicts with local coordinates?    
//    if(true){
//        if(system.get_z() > 0.6){
//            dVsdq -= -5*(0.5 - system.get_z())*system.Psi_z();
//        }
//        logTmp(system.get_z());
//        logTmp("Input: ", (0.5 - system.get_z())*system.Psi_z());
//    }

	// Return a linear gradient w.r.t. local coordinates
	return dVsdq;
}

Eigen::MatrixXd rPBC::Kv(System& system)
{
    /** @note Minv_dot * m cancels! */
    //return kq*Eigen::MatrixXd::Identity(system.n, system.n) + system.dMinv()*system.M();// + system.dPsi()*helpers::pseudoInverse(system.Psi());
    //logTmp("null_psi", null_psi);
   // logTmp("dnull_psi", helpers::pseudoInverse(nullPsi(system)) * dnullPsi(system));
//    logTmp("null_psi_pinv", helpers::pseudoInverse(null_psi));
//    logTmp("Kv", helpers::pseudoInverse(null_psi) * dnullPsi(system) + kappa*Eigen::MatrixXd::Identity(system.n, system.n));
    //Eigen::MatrixXd first_term = Eigen::MatrixXd::Zero(system.n, system.n);
    //first_term = helpers::pseudoInverse(nullPsi(system)) * dnullPsi(system);
    //logTmp("first_term", first_term);
    // Don forget local freedom here as well
    Eigen::MatrixXd result = kappa*Eigen::MatrixXd::Identity(system.n, system.n);
    if(false && has_local_freedom){
        result += helpers::pseudoInverse(nullPsi(system)) * dnullPsi(system);
    }
    
    return result;
}
//helpers::pseudoInverse(nullPsi(system)) * dnullPsi(system) + -> Deze cancelt cooperatief
// 
Eigen::MatrixXd& rPBC::pinvPsi(System& system)
{ 
    if(!pinv_psi_updated){
        
        pinv_psi = helpers::pseudoInverse(system.Psi().transpose());
        pinv_psi_updated = true;
    }
    
    return pinv_psi;
}

Eigen::MatrixXd& rPBC::nullPsi(System& system)
{ 
    if(!null_psi_updated){
        
        Eigen::FullPivLU<Eigen::MatrixXd> psi_lu(system.Psi().transpose());
        null_psi = psi_lu.kernel().transpose();
        null_psi_updated = true;
    }
    
    return null_psi;
}

Eigen::MatrixXd& rPBC::dnullPsi(System& system)
{
    if(!dnull_psi_updated){
        Eigen::MatrixXd dnull_psi_input = system.approximateDerivative(nullPsi(system), previous_null_psi);
        helpers::lowpassFilter(dnull_psi, dnull_psi_input, 0.95);
        dnull_psi_updated = true;
    }
    //logTmp("dm", dm);

    return dnull_psi;
}
Eigen::MatrixXd& rPBC::nullPinvPsi(System& system)
{
    logTmp("Warning not implemented function nullPinvPsi!");
    return null_psi;
}
