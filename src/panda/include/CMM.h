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
#include "EdgeIntegral.h"
#include "EdgeFlex.h"
#include "EdgeDelayFree.h"
#include "EdgeFlexDelayFree.h"
#include "CustomLog.h"
#include "Helpers.h"

#include "std_srvs/Empty.h"
#include "panda/getConnectionsOf.h"
#include "panda/isAgentLeader.h"

/* Possibly divide this up further in a system and a controller */

enum Status{
    STARTED,
    WAITING_FOR_OTHER,
    INIT_CMM,
    RUNNING
};

class CMM{

public:
	CMM(int set_id, int set_sampling_rate);
	~CMM();
    
    Status status = STARTED;

	Eigen::VectorXd sample(Eigen::VectorXd r_i);
	void resetIntegrators();
	void activateIntegral();
	void deactivateIntegral();
    void performHandshake();


    bool initEdges(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool retrieveEdges();

    bool hasInitialised();
	
private:

	unsigned char agent_id;
	int l, N;
	Eigen::MatrixXd gain, integral_gain;
	bool integral_enabled;
	double torque_enable;
	int sampling_rate;
    
    bool initialised = false;

	std::vector<Eigen::VectorXd> integral_states;

	ros::NodeHandle n;
	ros::Time initial_time;

	ros::ServiceClient connect_client, leader_client;
	ros::ServiceServer init_server;
	std::vector<std::unique_ptr<Edge>> edges;
	//std::vector<Edge*> edges;
	std::vector<std::unique_ptr<EdgeIntegral>> integral_edges;

};


#endif