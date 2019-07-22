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
#include "CustomLog.h"
#include "Helpers.h"

#include "panda/getConnectionsOf.h"

/* Possibly divide this up further in a system and a controller */

class CMM{

public:
	CMM(int set_id);
	~CMM();

	Eigen::VectorXd sample(Eigen::VectorXd r_i);
	
private:

	int agent_id, l, N;
	double gain, integral_gain;
	bool integral_enabled;

	std::vector<Eigen::VectorXd> integral_states;

	ros::NodeHandle n;

	ros::ServiceClient connect_client;
	std::vector<std::unique_ptr<Edge>> edges;
	std::vector<std::unique_ptr<Edge>> integral_edges;

	void initiateEdges();
};


#endif