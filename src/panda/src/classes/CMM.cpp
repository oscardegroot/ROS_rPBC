/*
File: CMM.cpp

Manages cooperative communication by managing a number of connections
*/

#include "CMM.h"


CMM::CMM(int set_id){

	logMsg("CMM", "Initiating..", 2);

	agent_id = set_id;

	//ros::NodeHandle nh;

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
	srand((unsigned int) agent_id + time(0));

	// Connect to the remote
	connect_client = n.serviceClient<panda::getConnectionsOf>("/get_connections_of");

	// Retrieve connections and create communication edges
	CMM::initiateEdges();

	initial_time = ros::Time::now();

	logMsg("CMM", "Done!", 2);
}

CMM::~CMM(){};

void CMM::initiateEdges(){

	// For all possible other agents
	for(int j = 0; j < N; j++){
		if(j != agent_id){
			
			// Ask the remote if we are connected
			panda::getConnectionsOf srv;
			srv.request.id_i = agent_id;
			srv.request.id_j = j;

			if(connect_client.call(srv)){
			//logTmp("Server is responding!");
				// If we are
				if(srv.response.is_connected){

					// Create a communication edge
					edges.push_back(std::make_unique<EdgeFlex>(agent_id, j, gain, l, false));
					
					if(integral_enabled){
						integral_edges.push_back(std::make_unique<EdgeIntegral>(agent_id, j, integral_gain, l, true));
						//integral_states.push_back(Eigen::VectorXd::Zero(l));
					}
				}
			}else{
				logMsg("CMM", "Failed to obtain formations from the server!", 0);
			}
			
		}
		
	}

	
	
}

Eigen::VectorXd CMM::sample(Eigen::VectorXd r){

	// Initialise the combined input
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(l);

	// Sample all edges
	for(int i = 0; i < edges.size(); i++){
		Eigen::VectorXd cur_tau = edges[i]->sample(r);
		tau += cur_tau;

		if(integral_enabled && ros::Time::now() - initial_time > ros::Duration(8.0) && cur_tau.transpose()*cur_tau < torque_enable && integral_edges[i]->is_activated == false){
			logMsg("CMM", "Integral Action Activated", 3);
			activateIntegral();
		}

		if(integral_enabled && integral_edges[i]->is_activated){
			//integral_states[i] += r;
			tau += integral_edges[i]->sample(r);
		}
	}

	// Return the combined input
	return tau;

}

void CMM::resetIntegrators(){

	if(!integral_enabled){
		return;
	}

	for(int i = 0; i < integral_edges.size(); i++){

		integral_edges[i]->reset();

	}
}

void CMM::activateIntegral(){

	if(!integral_enabled){
		return;
	}

	for(int i = 0; i < integral_edges.size(); i++){

		integral_edges[i]->activate();

	}
}

void CMM::deactivateIntegral(){

	if(!integral_enabled){
		return;
	}

	for(int i = 0; i < integral_edges.size(); i++){
		integral_edges[i]->deactivate();

	}
}