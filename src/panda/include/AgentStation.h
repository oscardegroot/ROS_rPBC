/*
File: Remote.h

A Remote node class that hosts services and implements an interface
*/


#ifndef Remote_H
#define Remote_H

#include "Station.h"
#include "Goals.h"

#include "std_msgs/Float64MultiArray.h"
#include "panda/getConnectionsOf.h"
#include "panda/isAgentLeader.h"
#include "panda/registerAgent.h"
#include "std_srvs/Empty.h"

#include "Exceptions.h"
#include "System.h"

/**
 * @class AgentStation
 * @author Oscar
 * @date 10/10/19
 * @file AgentStation.h
 * @brief Station that registers agents, constructs formations and transmits these to agents. Singleton design.
 */
class AgentStation : public Station{
    
public:
	AgentStation();
    
//    AgentStation(AgentStation& s) = delete;
    
    // Can be used from anywhere!
//    static AgentStation& getAgentStation()
//    {
//        static AgentStation instance; // Guaranteed to be destroyed, instantiated on first use
//        return instance;
//    }
    
    void starting() override;
    void update() override;
    void enableStation() override;
    void stopping() override;
    
    bool finishedRegistering() override;
    bool responsesReceived() const override;

    /**
     * @brief Return connections of an agent on the graph and associated gains
     */
	bool getConnectionsOf(panda::getConnectionsOf::Request &req, panda::getConnectionsOf::Response &res);

    /**
     * @brief Return if an agent is a leader and what its gain and reference is
     */
    bool isAgentLeader(panda::isAgentLeader::Request &req, panda::isAgentLeader::Response &res);

    /**
     * @brief Register an agent 
     */
    bool registerAgent(panda::registerAgent::Request &req, panda::registerAgent::Response &res);
    
    /**
     * @brief Callback to acknowledge an ACK message from a CMM
     */
    bool acknowledgeCMMReady(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    
    /**
     * @brief Workaround to plot agent signals from the start
     */
    void fakeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

private:

	int l, N;

    // Objectives
	std::unique_ptr<Goals> goals;
    
	// Stores all agent information (namespace / id / sampling_rate)
    std::vector<std::unique_ptr<Agent>> agents;
    
    // Acknowledged count
    unsigned int cmm_count = 0;
    
    // If this station runs a simulation
    bool is_simulation;

	// The server
	ros::ServiceServer connect_server, leader_server, register_server, cmm_server;
    std::vector<ros::ServiceClient> enable_clients;
    
    /**
     * @brief Startup the network by sending start messages to all CMM
     */
    void initiateNetwork();
    
    /**
     * @brief Pause gazebo physics if this is a simulation
     */
    void pauseGazebo();
    
        /**
     * @brief Unpause gazebo physics if this is a simulation
     */
    void unpauseGazebo();

};



#endif