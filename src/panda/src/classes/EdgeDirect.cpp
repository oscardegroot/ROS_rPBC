/*
File: Edge.cpp

Implements communication on an edge on one side using ST and WVM.
Externally calling sampleEdge will sample the buffer and return a wave to the other side of the channel.

i: agent id
j: connected agent id
gain: gain of the connection 
l_set: dimension of the channel
*/

#include "EdgeDirect.h"

EdgeDirect::EdgeDirect(Agent& agent, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set, int rate_mp_set)
	: Edge(agent, j, gain_set, l_set, r_star_set, rate_mp_set){

    potential = std::make_unique<NavigationFunction>(agent, l, r_star);
    
    // Counter that handles sampling rate converting (trigger initially)
    retrieval_counter = std::make_unique<helpers::Counter>(rate_mp, false);
}


/* Reconstruct data if necessary */
void EdgeDirect::applyReconstruction(Eigen::VectorXd& r_js, const Eigen::VectorXd& r_i){

    if(!retrieval_counter->trigger())
        return;
    
	if(!data_received)
	{
		r_js = r_i;	// Ensures zero output	
	}else{
		data_received = false;
	}
	
}

// Retrieve tau from the scattering transforma)tion
Eigen::VectorXd EdgeDirect::calculateControls(const Eigen::VectorXd& r_js, const Eigen::VectorXd& r_i){

	return -gain*(r_i-r_js);
}

// No waves, simply the output;
Eigen::VectorXd EdgeDirect::calculateWaves(const Eigen::VectorXd& s_in,	const Eigen::VectorXd& r_i){

    return r_i;
}
