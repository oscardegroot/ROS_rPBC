
#include "Agent.h";


Agent::Agent(){

	n = ros::NodeHandlePtr(new ros::NodeHandle("~"));

	// Get a nodehandle
	n->getParam("i_id", i_id);
	n->getParam("/l_dim", l);
	n->getParam("/N_dim", N);
	
	// Randomize the random seed
	srand((unsigned int) i_id + time(0));

	// Connect to the remote
	connect_client = n->serviceClient<panda::getConnectionsOf>("/get_connections_of");

	// Retrieve connections and create communication edges
	Agent::initiateEdges();

}		

Agent::~Agent(){
};

void Agent::initiateEdges(){

	// For all possible other agents
	for(int j = 0; j < N; j++){
		if(j != i_id){
			
			// Ask the remote if we are connected
			panda::getConnectionsOf srv;
			srv.request.id_i = i_id;
			srv.request.id_j = j;

			if(connect_client.call(srv)){

				// If we are
				if(srv.response.is_connected){

					// Create a communication edge
					edges.push_back(new Edge(i_id, j, 1.0, l));
				}
			}else{
				std::cout << "[Error]: Failed to call the server!\n";
			}
			
		}
		
	}

	std::cout << "Edges initiated!\n";
	
}


void Agent::sample(){
	// For testing purposes, generate an r and randomize the seed
	Eigen::VectorXd r_i(l);
	
	// Sample all edges
	for(int i = 0; i < edges.size(); i++){
		r_i = Eigen::VectorXd::Random(l);
		edges[i]->sampleEdge(r_i);
	}

}
