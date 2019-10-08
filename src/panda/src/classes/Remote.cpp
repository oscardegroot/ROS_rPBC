/*
File: Remote.cpp

A Remote node class that hosts services and implements an interface
*/


#include "Remote.h"

Remote::Remote(){

	helpers::safelyRetrieve(n, "/l", l);
	helpers::safelyRetrieve(n, "/N_agents", N);

	goals = std::make_unique<Goals>();

    status = REGISTERING_AGENTS;
    
	// Start servers
	connect_server = n.advertiseService("/getConnectionsOf", &Remote::getConnectionsOf, this);
    leader_server = n.advertiseService("/isAgentLeader", &Remote::isAgentLeader, this);
    register_server = n.advertiseService("/registerAgent", &Remote::registerAgent, this);
    cmm_server = n.advertiseService("/cmmReady", &Remote::acknowledgeCMMReady, this);

    // Workaround!
    ros::Subscriber topic_create = n.subscribe("/agent_z", 1, &Remote::fakeCallback, this);

    // 2 second timer
    timer = std::make_unique<helpers::SimpleTimer>(4.0);
    
	logMsg("Remote", "Formation server initiated", 2);

}

Remote::~Remote(){
    logMsg("Remote", "Remote destroyed", 1);
}


/** @Workaround to claim existence of agent_z when the plots are launched... */
void Remote::fakeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    return;
}


void Remote::initiateNetwork(){


    for(int i = 0; i < agents.size(); i++){
        
        ros::ServiceClient init_client = n.serviceClient<std_srvs::Empty>("/agent" + std::to_string(agents[i]->getID()) +
                "/initEdges");

        std_srvs::Empty srv;
        
        // Declare a server call lambda to be repeated
        auto server_call = [&](){ 
                return init_client.call(srv);
            };
        
        // Call the service until it works or until a second has passed
        helpers::repeatedAttempts(server_call, 1.0, "Failed to start the network of agent " + agents[i]->getType() + "!");
        
        logMsg("Remote", "Network of agent " + agents[i]->getType() + " started.", 2);
    }
    
}

bool Remote::getConnectionsOf(panda::getConnectionsOf::Request &req, panda::getConnectionsOf::Response &res){

	Eigen::VectorXd r_star = Eigen::VectorXd::Zero(l);
	res.is_connected = goals->formation->retrieveConnectionBetween(req.id_i, req.id_j, r_star);
    
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

// Register an agent to this remote
bool Remote::registerAgent(panda::registerAgent::Request &req, panda::registerAgent::Response &res){

    // Check if the agent doesn't already exist
    for(int i = 0; i < agents.size(); i++){
        if(agents[i]->getID() == req.id){

            logMsg("Remote", "Agent trying to register already exists!", 0);
            return false;
        }
    }

    // Add the received agent to the list
    agents.push_back(std::make_unique<Agent>(req.id, req.sampling_rate, req.type));

    logMsg("Remote", "Added new agent of type " + agents[agents.size() - 1]->getType() + " with ID " +
    std::to_string(agents[agents.size() - 1]->getID()) + ".", 2);

    return true;
}

/** @brief Count the number of agents ready to go */
bool Remote::acknowledgeCMMReady(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

    // Simply count the number of cmms ready
    cmm_count++;

    return true;
}

void Remote::pauseGazebo(){
    
    //ScopeTimer timer("Pausing Gazebo");
    auto pause_client = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    std_srvs::Empty srv;
        
    // Declare a server call lambda to be repeated
    auto pause_call = [&](){ 
            return pause_client.call(srv);
        };
        
    // Call the service until it works or until a second has passed
    helpers::repeatedAttempts(pause_call, 4.0, "Failed to pause gazebo!");
    
    //logMsg("Remote", "Gazebo Physics Paused.", 1);
    
}

void Remote::unpauseGazebo(){
    
    auto pause_client = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    std_srvs::Empty srv;

    if(!pause_client.call(srv)){
        
        throw OperationalException("Gazebo could not be unpaused!");
    }
    
}

/** @brief If timer is done, stop registering */
bool Remote::finishedRegistering(){
    return timer->finished();
}

/** @brief If a message was received from all CMM, proceed to control */
bool Remote::cmm_ready()
{
    return (cmm_count == agents.size());
}
