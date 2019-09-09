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
    status = REGISTERING_AGENTS;

	// Start a server
	connect_server = n.advertiseService("/getConnectionsOf", &Remote::getConnectionsOf, this);
    leader_server = n.advertiseService("/isAgentLeader", &Remote::isAgentLeader, this);
    register_server = n.advertiseService("/registerAgent", &Remote::registerAgent, this);

	logMsg("Remote", "Formation server initiated", 2);

}

Remote::~Remote(){
    logMsg("Remote", "Remote destroyed", 1);
}

void Remote::initiateNetwork(){


    for(int i = 0; i < agents.size(); i++){
        ros::ServiceClient init_client = n.serviceClient<std_srvs::Empty>("/agent" + std::to_string(agents[i].ID) +
                "/initEdges");


        std_srvs::Empty srv;
        
        long attempts = 0;
        while(!init_client.call(srv)){
            
            attempts++;
            if(attempts > MAX_HANDSHAKE_ATTEMPTS){
                throw RegisteringException("Failed to start the network of agent " +
            agents[i].name + "!");
            }
            
            
        }
        
        logMsg("Remote", "Network of agent " + agents[i].name + " started.", 2);

    }
    
    status = SPINNING;

}

bool Remote::getConnectionsOf(panda::getConnectionsOf::Request &req, panda::getConnectionsOf::Response &res){

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

/// Register an agent to this remote
bool Remote::registerAgent(panda::registerAgent::Request &req, panda::registerAgent::Response &res){

    // Check if the agent doesn't already exist
    for(int i = 0; i < agents.size(); i++){
        if(agents[i].ID == req.id){

            logMsg("Remote", "Agent trying to register already exists!", 0);
            return false;
        }
    }

    // Add the received agent to the list
    Agent new_agent({req.type, req.name, req.id, req.sampling_rate});
    agents.push_back(new_agent);

    logMsg("Remote", "Added new agent of type " + new_agent.name + " with ID " +
    std::to_string(new_agent.ID) + ".", 2);
//
    if(agents.size() == N){
        ready = true;
    }

    return true;
}

bool Remote::isReady(){
    return ready;
}