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
    
	Eigen::VectorXd gain_e;
	helpers::safelyRetrieveEigen(n, "/network_gain", gain_e, l);
	gain = Eigen::MatrixXd(gain_e.asDiagonal());

	// Connect to the remote
	connect_client = n.serviceClient<panda::getConnectionsOf>("/getConnectionsOf");
    leader_client = n.serviceClient<panda::isAgentLeader>("/isAgentLeader");
    init_server = n.advertiseService("/agent" + std::to_string(agent_id) + "/initEdges", &CMM::initEdges, this);

	// Retrieve connections and create communication edges
    status = WAITING_FOR_OTHER;

    performHandshake();
    
	logMsg("CMM", "Done!", 2);
}

CMM::~CMM(){};

// Only initialise when all systems have registered
void CMM::performHandshake(){
    
	ros::Rate loop_rate(100);
    while(ros::ok() && status != RUNNING){
        
        switch(status){
            // If registration is not complete, wait for the message from the server
            case(WAITING_FOR_OTHER):
                ros::spinOnce();
                break;
                
            // If the message was received, initialise!
            case INIT_CMM: 
                logMsg("CMM", "Initiating network call from Remote was received, processing now...", 2);
                retrieveConnections();
                break;
        }

        loop_rate.sleep();
    }
}

bool CMM::retrieveConnections(){
    
    // Retrieve leader info from the server and create edges if necessary
    setupLeader();
    
    // Retrieve connections to other agents from the server and create edges if necessary
	setupEdges();
    
    // Transition to the running state
    status = RUNNING;
}

void CMM::setupLeader(){
    
    panda::isAgentLeader srv;
    srv.request.id = agent_id;
    
    // Call for leader information
    if(leader_client.call(srv)){
        if(srv.response.is_leader){
            
            // Parse the retrieved vectors
            Eigen::VectorXd temp_gain = helpers::messageToEigen(srv.response.gain, l);
            Eigen::VectorXd temp_ref = helpers::messageToEigen(srv.response.ref, l);
            Eigen::MatrixXd leader_gain = Eigen::MatrixXd(temp_gain.asDiagonal());
            
            // Create an edge
            edges.push_back(std::make_unique<EdgeLeader>(agent_id, -1, leader_gain, l, temp_ref));
        }
    }
}

void CMM::setupEdges(){
    
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
                    
                    // Retrieve the formation data
                    Eigen::VectorXd r_star = helpers::vectorToEigen(srv.response.r_star.data);

					// Create a communication edge
					edges.push_back(std::make_unique<EdgeFlex>(agent_id, j, gain, l, r_star, rate_mp));
					
				}
			}else{
				logMsg("CMM", "Failed to obtain formations from the server!", 0);
			}
		}
	}
}

bool CMM::initEdges(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

    // Proceed to initialisation
    status = INIT_CMM;
    
    return true;
}

Eigen::VectorXd CMM::sample(Eigen::VectorXd r){

	// Initialise the combined input
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(l);

	// Sample all edges
	for(int i = 0; i < edges.size(); i++){
		tau += edges[i]->sample(r);
    }
    
	// Return the combined input
	return tau;
}

bool CMM::hasInitialised() const
{
    return initialised;
}