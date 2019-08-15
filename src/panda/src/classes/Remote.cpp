/*
File: Remote.cpp

A Remote node class that hosts services and implements an interface
*/


#include "Remote.h"

Remote::Remote(){

	helpers::safelyRetrieve(n, "/l", l);
	helpers::safelyRetrieve(n, "/N_agents", N);

	goals = std::make_unique<Goals>();
	goals->Init(N, l);

	// Start a server
	connect_server = n.advertiseService("/get_connections_of",
				 &Remote::retrieveConnectionsOf, this);
    leader_server = n.advertiseService("/isAgentLeader",
                                        &Remote::isAgentLeader, this);

	logMsg("Remote", "Formation server initiated", 2);

}

Remote::~Remote(){

}

bool Remote::retrieveConnectionsOf(panda::getConnectionsOf::Request &req, panda::getConnectionsOf::Response &res){
	
	Eigen::VectorXd r_star = Eigen::VectorXd::Zero(l);
	res.is_connected = goals->retrieveConnectionBetween(req.id_i, req.id_j, r_star);

	// Convert from vector to msg
	std_msgs::Float64MultiArray r_star_vec;

    r_star_vec.data.resize(l);
	for(int i = 0; i < l; i++){
        r_star_vec.data[i] = r_star(i);
	}

	res.r_star = r_star_vec;

	logMsg("Remote", "Responded to connection request between " + std::to_string(req.id_i) +
	" and " + std::to_string(req.id_j) + ", r* = " + std::to_string(r_star(0)) + ", " + std::to_string(r_star(1)), 2);


	return true;
}


bool Remote::isAgentLeader(panda::isAgentLeader::Request &req, panda::isAgentLeader::Response &res){

    Eigen::VectorXd reference;
    Eigen::VectorXd gain;

    res.is_leader = goals->isAgentLeader(req.id, reference, gain);
    if(res.is_leader) {
        res.ref = helpers::eigenToMessage(reference);
        res.gain = helpers::eigenToMessage(gain);
    }
    logMsg("Remote", "Responded to is_leader request of agent " + std::to_string(req.id), 2);

    return true;

}