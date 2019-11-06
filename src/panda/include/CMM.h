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

/* Possibly divide this up further in a system and a controller */

enum Status{
    STARTED = 0,
    WAITING_FOR_OTHER,
    INIT_CMM,
    RUNNING
};

class CMM{

public:
	CMM(std::string agent_type);
	~CMM();
    
    Status status;
    
    //The contained agent
    std::unique_ptr<Agent> agent;
    
	Eigen::VectorXd sample(const Eigen::VectorXd& r_i);
    
    /** @brief Wait for the server message to retrieve the network */
    void performHandshake();

    bool initEdges(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool addObstacle(panda::addObstacle::Request& req, panda::addObstacle::Response& res);
    
    bool retrieveConnections();
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

	//unsigned char agent_id;
	int l, N;
	Eigen::MatrixXd gain;
    
    int rate_mp;
    
    bool initialised = false;

	ros::NodeHandle nh;

	ros::ServiceClient connect_client, leader_client;
	ros::ServiceServer init_server, obstacle_server;
	std::vector<std::unique_ptr<Edge>> edges;
    std::unique_ptr<Selector> coop_selector, leader_selector;

};


#endif