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
	helpers::safelyRetrieve(n, "/network_gain", gain);
	
	// Randomize the random seed
	srand((unsigned int) agent_id + time(0));

	// Connect to the remote
	connect_client = n.serviceClient<panda::getConnectionsOf>("/get_connections_of");

	// Retrieve connections and create communication edges
	CMM::initiateEdges();

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
					edges.push_back(std::make_unique<Edge>(agent_id, j,gain, l));
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
		tau += edges[i]->sampleEdge(r);
	}

	// Return the combined input
	return tau;

}