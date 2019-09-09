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
	//setLine(0.08, 0.0);

	ros::NodeHandle nh;

	initLeaders(nh);
}

Goals::~Goals(){};

// A function that sets a zero formation
void Goals::setConsensus(){

    connections = {};

	for(int i = 0; i < N; i++){

		std::vector<connection> connections_i;

		for (int j = 0; j < N; j++){
			
			// If agent i connects to j (and check for equal indices)
			if(graph[i][j] && i != j){

				//connection
				connections_i.push_back({j, Eigen::VectorXd::Zero(l)});
			}
		}

		connections.push_back(connections_i);
	}

	logMsg("Goals", "Goal is set to consensus");
}

void Goals::setCircle(const double radius, const double phase_offset){
    connections = {};

    Eigen::Matrix<double, 2, 1> positions[N];

    // Determine positions
    double phase = phase_offset;
    for (int i = 0; i < N; i++){
        positions[i] = Eigen::Matrix<double, 2, 1>::Zero();
        positions[i] << std::cos(phase)*radius,
                        std::sin(phase)*radius;

        phase += (2.0 * M_PI) / double(N);
    }

    // Set connections
    for(int i = 0; i < N; i++){

        std::vector<connection> connections_i;

        for (int j = 0; j < N; j++){

            // If agent i connects to j (and check for equal indices)
            if(graph[i][j] && i != j){

                // Calculate the difference and divide by two for ST
                Eigen::Matrix<double, 2, 1> r_jsi = (positions[j] - positions[i])/2.0;

                //connection
                connections_i.push_back({j, r_jsi});
            }
        }
        connections.push_back(connections_i);
    }

    logMsg("Goals", "Goal is set to circle with radius " + std::to_string(radius)  + ".", 2);

}

void Goals::setLine(const double spacing, const double angle){
    connections = {};

    Eigen::Matrix<double, 2, 1> positions[N];

    double length = double(N)*spacing;
    // Determine positions
    for (int i = 0; i < N; i++){
        positions[i] = Eigen::Matrix<double, 2, 1>::Zero();
        positions[i] << std::cos(angle)*(-length/2.0 + double(i)*spacing),
                        std::sin(angle)*(-length/2.0 + double(i)*spacing);
    }

    // Set connections
    for(int i = 0; i < N; i++){

        std::vector<connection> connections_i;

        for (int j = 0; j < N; j++){

            // If agent i connects to j (and check for equal indices)
            if(graph[i][j] && i != j){

                // Calculate the difference and divide by two for ST
                Eigen::Matrix<double, 2, 1> r_jsi = (positions[j] - positions[i])/2.0;

                //connection
                connections_i.push_back({j, r_jsi});
            }
        }
        connections.push_back(connections_i);
    }

    logMsg("Goals", "Goal is set to line with spacing " + std::to_string(spacing)  + ".", 2);

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

void Goals::initLeaders(ros::NodeHandle & nh){

    leaders = {};
    int id;
    Eigen::VectorXd gain, ref;

    try {
        helpers::safelyRetrieve(nh, "/leader/id", id);
        helpers::safelyRetrieveEigen(nh, "/leader/gain", gain, l);
        helpers::safelyRetrieveEigen(nh, "/leader/ref", ref, l);

        leaders.push_back({id, ref, gain});
    }
    catch(RetrievalException exc){
        return;
    }

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

bool Goals::isAgentLeader(u_int id, Eigen::VectorXd & ref, Eigen::VectorXd & gain){

    for(int i = 0; i < leaders.size(); i++){
        if(leaders[i].id == id){
            ref = leaders[i].ref;
            gain = leaders[i].gain;
            return true;
        }
    }

    return false;

}
