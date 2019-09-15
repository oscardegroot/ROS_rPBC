/*
File: CMM.cpp

Manages cooperative communication by managing a number of connections
*/

#include "CMM.h"


CMM::CMM(int set_id, int set_sampling_rate){

	logMsg("CMM", "Initiating..", 2);

	agent_id = set_id;
	sampling_rate = set_sampling_rate;
    
    int network_rate;
    helpers::safelyRetrieve(n, "/sampling_rate", network_rate);
    
    // Check if parameters are correct
    if(sampling_rate % network_rate != 0){
        throw ParameterException("Sampling rate of agent " + std::to_string(set_id) + " is not a multiple of the network rate!");
    }
    
    // Calculate the rate multiplier
    rate_mp = sampling_rate / network_rate;
    
	// Retrieve parameters
	helpers::safelyRetrieve(n, "/l", l);
	helpers::safelyRetrieve(n, "/N_agents", N);

    helpers::safelyRetrieve(n, "/controller/integral/enabled", integral_enabled, false);

    if(integral_enabled) {
        Eigen::VectorXd gain_i;
        helpers::safelyRetrieveEigen(n, "/controller/integral/gain", gain_i, l);
        integral_gain = Eigen::MatrixXd(gain_i.asDiagonal());

        helpers::safelyRetrieve(n, "/controller/integral/torque_enable", torque_enable, 4.0);
    }

	Eigen::VectorXd gain_e;
	helpers::safelyRetrieveEigen(n, "/network_gain", gain_e, l);
	gain = Eigen::MatrixXd(gain_e.asDiagonal());

	// Randomize the random seed
	//srand((unsigned int) agent_id + time(0));

	// Connect to the remote
	connect_client = n.serviceClient<panda::getConnectionsOf>("/getConnectionsOf");
    leader_client = n.serviceClient<panda::isAgentLeader>("/isAgentLeader");
    init_server = n.advertiseService("/agent" + std::to_string(agent_id) + "/initEdges", &CMM::initEdges, this);

	// Retrieve connections and create communication edges
	//CMM::initiateEdges();
	initial_time = ros::Time::now();
    status = WAITING_FOR_OTHER;

    performHandshake();
	logMsg("CMM", "Done!", 2);
}

CMM::~CMM(){};

void CMM::performHandshake(){
    
    //logTmp("Performing Handshake Now");
	ros::Rate loop_rate(100);
    while(ros::ok() && status != RUNNING){
        
        switch(status){
            case(WAITING_FOR_OTHER):
                ros::spinOnce();
                break;
                
            case INIT_CMM: 
                logMsg("CMM", "Initiating network call from Remote was received, processing now...", 2);
                retrieveEdges();
                
                break;
                
        }

        loop_rate.sleep();
    }
}

bool CMM::retrieveEdges(){
    panda::isAgentLeader srv;
    srv.request.id = agent_id;
    if(leader_client.call(srv)){
        if(srv.response.is_leader){
            Eigen::VectorXd temp_gain = helpers::messageToEigen(srv.response.gain, l);
            Eigen::VectorXd temp_ref = helpers::messageToEigen(srv.response.ref, l);
            Eigen::MatrixXd leader_gain = Eigen::MatrixXd(temp_gain.asDiagonal());
            edges.push_back(std::make_unique<EdgeLeader>(agent_id, -1, leader_gain, l, temp_ref));
        }
    }

	// For all possible other agents
	for(int j = 0; j < N; j++){
		if(j != agent_id){
			
			// Ask the remote if we are connected
			panda::getConnectionsOf srv;
			srv.request.id_i = agent_id;
			srv.request.id_j = j;

			if(connect_client.call(srv)){

				// If we are
				if(srv.response.is_connected){

                    std::vector<double> r_star_array = srv.response.r_star.data;

                    Eigen::VectorXd r_star = Eigen::VectorXd::Zero(l);
                    for(int i = 0; i < l; i++){

                        r_star(i, 0) = r_star_array[i];
                    }

					// Create a communication edge
					edges.push_back(std::make_unique<EdgeFlex>(agent_id, j, gain, l, r_star, rate_mp));
					
//					if(integral_enabled){
//						//integral_edges.push_back(std::make_unique<EdgeIntegral>(agent_id, j, integral_gain, l, true));
//						//integral_states.push_back(Eigen::VectorXd::Zero(l));
//					}
				}
			}else{
				logMsg("CMM", "Failed to obtain formations from the server!", 0);
			}
			
		}
		
	}
    
    status = RUNNING;

}

bool CMM::initEdges(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    //edges = {}; todo: instead delete every element
    status = INIT_CMM;
    return true;
    
	
}

Eigen::VectorXd CMM::sample(Eigen::VectorXd r){

	// Initialise the combined input
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(l);

	// Sample all edges
	for(int i = 0; i < edges.size(); i++){
		Eigen::VectorXd cur_tau = edges[i]->sample(r);
		tau += cur_tau;

//		if(integral_enabled && ros::Time::now() - initial_time > ros::Duration(8.0) && cur_tau.transpose()*cur_tau < torque_enable && integral_edges[i]->is_activated == false){
//			logMsg("CMM", "Integral Action Activated", 3);
//			activateIntegral();
//		}
//
//		if(integral_enabled && integral_edges[i]->is_activated){
//			//integral_states[i] += r;
//			tau += integral_edges[i]->sample(r);
//		}
	}

	// Return the combined input
	return tau;

}

bool CMM::hasInitialised()
{
    return initialised;
}


//
//void CMM::resetIntegrators(){
//
//	if(!integral_enabled){
//		return;
//	}
//
//	for(int i = 0; i < integral_edges.size(); i++){
//
//		integral_edges[i]->reset();
//
//	}
//}
//
//void CMM::activateIntegral(){
//
//	if(!integral_enabled){
//		return;
//	}
//
//	for(int i = 0; i < integral_edges.size(); i++){
//
//		integral_edges[i]->activate();
//
//	}
//}
//
//void CMM::deactivateIntegral(){
//
//	if(!integral_enabled){
//		return;
//	}
//
//	for(int i = 0; i < integral_edges.size(); i++){
//		integral_edges[i]->deactivate();
//
//	}
//}

