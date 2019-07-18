/*
File: Goals.cpp

Contains a number of functions describing formations and other cooperative goals
*/

#include "Goals.h"

// The construct takes the number of agents
Goals::Goals(){};


void Goals::Init(int N_set, int l_dim){
	N = N_set;
	l = l_dim;

	setGraphFC();
	setConsensus();
}

Goals::~Goals(){};

// A function that sets a zero formation
void Goals::setConsensus(){
	
	for(int i = 0; i < N; i++){

		std::vector<connection> connections_i;

		for (int j = 0; j < N; j++){
			
			
			// If agent i connects to j (and check for equal indices)
			if(graph[i][j] && i != j){

				//connection 

				connections_i.push_back({
					j,
					Eigen::VectorXd::Zero(l)
				});
			}
		}

		connections.push_back(connections_i);
	}

	logMsg("Goals", "Goal is set to consensus");
}

void Goals::setGraphFC(){

	std::vector<bool> c_i;
	c_i.assign(N, true);

	// For every agent
	for(int i = 0; i < N; i++){
		graph.push_back(c_i);
	}

	logMsg("Goals", "Graph is set to fully connected");
}

// Set a formation with even distances between agents (circular)
void Goals::setFormationEven(double d){

}

	
std::vector<connection> Goals::retrieveConnectionsOf(int agent_id){
	return connections[agent_id];
}

bool Goals::retrieveConnectionBetween(u_int id_i, u_int id_j, Eigen::VectorXd & r_star){
	
	std::vector<connection> connects = retrieveConnectionsOf(id_i);

	for(int j = 0; j < N; j++){

		if(connects[j].id == id_j){
			r_star = connects[j].r_star;
			return true;
		}
	}

	return false;
}
