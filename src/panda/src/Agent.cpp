#include "Agent.h"



void initiateEdges(){

	for(int j = 0; j < N; j++){
		if(j != agent_id){
			
			panda::getConnectionsOf srv;
			srv.request.id_i = agent_id;
			srv.request.id_j = j;

			if(connect_client.call(srv)){

				if(srv.response.is_connected){

					edges.push_back(new Edge(agent_id, j, 1.0, l));
					//edges.emplace_back(agent_id, j, 1.0, l);
				}
			}else{
				std::cout << "[Error]: Failed to call the server!\n";
			}
			
		}
		
	}

	std::cout << "Edges initiated!\n";
	
}



int main(int argc, char **argv){
	
	// Initialise ROS
	ros::init(argc, argv, "Agent");

	// Get a nodehandle
	ros::NodeHandle n("~");
	n.getParam("i_id", agent_id);
	n.getParam("/l_dim", l);
	n.getParam("/N_dim", N);
	srand((unsigned int) agent_id + time(0));
	// Connect to the remote
	connect_client = n.serviceClient<panda::getConnectionsOf>("/get_connections_of");

	// Retrieve connections and create communication edges
	initiateEdges();
	
	// For testing purposes, generate an r
	Eigen::VectorXd r_i(l);
	
	ros::Rate loop_rate(0.5);

	while(ros::ok()){
		
		
		// Sample all edges
		for(int i = 0; i < edges.size(); i++){
			r_i = Eigen::VectorXd::Random(l);
			edges[i]->sampleEdge(r_i);
		}

		ros::spinOnce();
		// ros::Duration(0.1).sleep();
		// ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;

}