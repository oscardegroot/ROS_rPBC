/*
File: Remote.cpp

A Remote node class that hosts services and implements an interface
*/


#include "AgentStation.h"

AgentStation::AgentStation(){

	helpers::safelyRetrieve(nh, "/l", l);
	helpers::safelyRetrieve(nh, "/N_agents", N);

	goals = std::make_unique<Goals>();
    
    helpers::safelyRetrieve(nh, "/is_simulation", is_simulation);
    
	// Start servers
	connect_server = nh.advertiseService("/getConnectionsOf", &AgentStation::getConnectionsOf, this);
    leader_server = nh.advertiseService("/isAgentLeader", &AgentStation::isAgentLeader, this);
    register_server = nh.advertiseService("/registerAgent", &AgentStation::registerAgent, this);
    cmm_server = nh.advertiseService("/cmmReady", &AgentStation::acknowledgeCMMReady, this);

    // Workaround!
    for(int i = 0; i < N; i++){
        ros::Subscriber topic_create = nh.subscribe("/agents/" + std::to_string(i) + "/z", 1, &AgentStation::fakeCallback, this);
    }

    pauseGazebo();
    
	logMsg("Remote", "AgentStation intialisation completed", 2);

}

void AgentStation::starting(){
    
    pauseGazebo();
    initiateNetwork();
    pauseGazebo();
}

void AgentStation::update(){
    
}

void AgentStation::stopping(){
    
    logMsg("Agent Station", "Agent station shutting down", 2);
}


/** @Workaround to claim existence of agent_z when the plots are launched... */
void AgentStation::fakeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    return;
}


void AgentStation::initiateNetwork(){

    logMsg("Agent Station", "Initiating Agent Network...", 2);
    for(int i = 0; i < agents.size(); i++){
        logMsg("Agent Station", "Initiating Agent " + std::to_string(agents[i]->getID()), 2);

        ros::ServiceClient init_client = nh.serviceClient<std_srvs::Empty>("/agent" + std::to_string(agents[i]->getID()) +
                "/initEdges");

        std_srvs::Empty srv;
        
        // Declare a server call lambda to be repeated
        auto server_call = [&](){ 
                return init_client.call(srv);
            };
        
        // Call the service until it works or until a second has passed
        helpers::repeatedAttempts(server_call, 2.0, "Failed to start the network of agent " + agents[i]->getType() + "!");
        
        logMsg("Remote", "Network of agent " + agents[i]->getType() + " started.", 2);
    }
    
}


bool AgentStation::getConnectionsOf(panda::getConnectionsOf::Request &req, panda::getConnectionsOf::Response &res){

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
	" and " + std::to_string(req.id_j) + ", (in 3D) r* = " + std::to_string(r_star(0)) + ", " + std::to_string(r_star(1)) + ", " + std::to_string(r_star(2)), 2);


	return true;
}


bool AgentStation::isAgentLeader(panda::isAgentLeader::Request &req, panda::isAgentLeader::Response &res){

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
bool AgentStation::registerAgent(panda::registerAgent::Request &req, panda::registerAgent::Response &res){

    // Check if the agent doesn't already exist
    for(int i = 0; i < agents.size(); i++){
        if(agents[i]->getID() == req.id){

            logMsg("Remote", "Agent trying to register already exists!", 0);
            return false;
        }
    }

    // Add the received agent to the list
    agents.push_back(std::make_unique<Agent>(req.id, req.sampling_rate, req.type));
    enable_clients.push_back(nh.serviceClient<std_srvs::Empty>("/Agent_" + std::to_string(req.id) + "/enable"));

    logMsg("Remote", "Added new agent of type " + agents[agents.size() - 1]->getType() + " with ID " +
    std::to_string(agents[agents.size() - 1]->getID()) + ".", 2);

    return true;
}

/** @brief Count the number of agents ready to go */
bool AgentStation::acknowledgeCMMReady(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

    // Simply count the number of cmms ready
    cmm_count++;

    return true;
}

void AgentStation::pauseGazebo(){
    
    if(!is_simulation){
        return;
    }
    
    //ScopeTimer timer("Pausing Gazebo");
    auto pause_client = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    std_srvs::Empty srv;
        
    // Declare a server call lambda to be repeated
    auto pause_call = [&](){ 
            return pause_client.call(srv);
        };
        
    // Call the service until it works or until a second has passed
    helpers::repeatedAttempts(pause_call, 4.0, "Failed to pause gazebo!");
}

void AgentStation::unpauseGazebo(){
    
    if(!is_simulation){
        return;
    }
    
    auto pause_client = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    std_srvs::Empty srv;

    if(!pause_client.call(srv)){
        
        throw OperationalException("Gazebo could not be unpaused!");
    }
    
}

/** @brief If a message was received from all CMM, proceed to control */
bool AgentStation::responsesReceived() const {
    return (cmm_count == agents.size());
}

bool AgentStation::finishedRegistering(){
    pauseGazebo();
    return Station::finishedRegistering();
}

void AgentStation::enableStation()
{
    unpauseGazebo();
    std_srvs::Empty srv;
    for(auto& client : enable_clients){
        
        client.call(srv);
    }
}
