
#include "Agent.h"

// Constructor for client side agents
Agent::Agent(std::string type_name_set)
    : type_name(type_name_set)
{
    //logTmp("node name", ros::this_node::getName()); //= /agent_0
    nh_private = std::make_unique<ros::NodeHandle>(ros::this_node::getName()); // Declare a private nodehandle

    retrieveParameter("ID", ID);
    retrieveParameter("sampling_rate", sampling_rate);

    // Register this agent
    registerToServer();
}

// Constructor for server side agent (no registering!)
Agent::Agent(int ID_set, int sampling_rate_set, std::string type_name_set)
    : ID(ID_set), sampling_rate(sampling_rate_set), type_name(type_name_set)
{
    nh_private = nullptr; // Remote declaration
}

Agent::~Agent(){
    if(nh_private){
        logMsg("Agent", getType() + " shutting down.", 2);
    }
    //logAssert(nh_private == nullptr, "Agent is owned by the Remote!");
}

// Register this agent to the server
void Agent::registerToServer()
{
    // Setup a client
    ros::NodeHandle nh_global;
    ros::ServiceClient register_client = nh_global.serviceClient<panda::registerAgent>("/registerAgent");
    
    // This could be the issue
    /** Previous: one call */
    // Define the request
//    auto srv = agentToSrv();
//
//    // Call the service
//    if(register_client.call(srv)){
//        logMsg("Agent", "Registered " + getType() + " succesfully", 2);
//    }else{
//        throw RegisteringException("Agent could not register!");
//    }
    
    /* Attempt to register until the remote responds or the attempt times out */
    auto srv = agentToSrv();
        
    // Declare a server call lambda to be repeated
    auto register_call = [&](){ 
            return register_client.call(srv);
        };
    
    // Call the service until it works or until a second has passed
    helpers::repeatedAttempts(register_call, 1.0, "Agent " + getType() + "could not register!");
    
    logMsg("Agent", "Registered " + getType() + " succesfully", 2);
}

// Convert this agents parameters to a service call
panda::registerAgent Agent::agentToSrv(){
        panda::registerAgent srv;
        
        srv.request.type = type_name;
        srv.request.name = node_name;
        srv.request.id = ID;
        srv.request.sampling_rate = sampling_rate;

        return srv;
}

