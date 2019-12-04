/*
File: CMM.h

Manages cooperative communication by managing a number of connections
*/

#ifndef CMM_H
#define CMM_H

#include "ros/ros.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <memory>

#include "Edge.h"
#include "EdgeDirect.h"
#include "EdgeLeader.h"
#include "EdgeFlex.h"
#include "EdgeDelayFree.h"
#include "EdgeFlexDelayFree.h"
#include "CustomLog.h"
#include "Helpers.h"
#include "Agent.h"
#include "Selector.h"

#include "std_srvs/Empty.h"
#include "panda/getConnectionsOf.h"
#include "panda/isAgentLeader.h"
#include "panda/addObstacle.h"

// FSM Status enumerate
enum Status{
    STARTED = 0,
    WAITING_FOR_OTHER,
    INIT_CMM,
    RUNNING
};

/**
 * @class CMM
 * @author Oscar
 * @file CMM.h
 * @brief Class that centrally combines edges to other agents such that cooperative inputs may be centrally sampled.
 */
class CMM{

public:
	CMM(std::string agent_type);
	~CMM();
    
    // Internal FSM status
    Status status;
    
    //The contained agent
    std::unique_ptr<Agent> agent;
    
	Eigen::VectorXd sample(const Eigen::VectorXd& r_i);
    
    /** @brief Wait for the server message to retrieve the network */
    void performHandshake();

    /** @brief Service call that initiates the creation of edges */
    bool initEdges(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    
    // Function for adding an obstacle to all edges
    bool addObstacle(panda::addObstacle::Request& req, panda::addObstacle::Response& res);
    
    // Retrieve connections from the server
    bool retrieveConnections();
    
    // Setup edges
    void setupLeader();
    void setupEdges();

    bool hasInitialised() const;
    
    /**
     * @brief Getters for cooperative dimensions
     */
    int coopDim() const { return coop_selector->dim();};
    int leaderDim() const { return leader_selector->dim();};
    int allDim() const { return coop_selector->dim() + leader_selector->dim();};
    

private:

	int l, N;
	Eigen::MatrixXd gain;
    
    // Sampling rate ratio to the network
    int rate_mp;
    
    bool initialised = false;

	ros::NodeHandle nh;

	ros::ServiceClient connect_client, leader_client;
	ros::ServiceServer init_server, obstacle_server;
	std::vector<std::unique_ptr<Edge>> edges;
    std::unique_ptr<Selector> coop_selector, leader_selector;

};


#endif