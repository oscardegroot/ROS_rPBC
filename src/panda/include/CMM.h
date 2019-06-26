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

#include "panda/getConnectionsOf.h"



/* Possibly divide this up further in a system and a controller */

class CMM{

public:
	CMM();
	~CMM();

	Eigen::VectorXd sample(Eigen::VectorXd r_i);
	
private:

	int i_id, l, N;
	double gain;

	ros::NodeHandlePtr n;

	ros::ServiceClient connect_client;
	std::vector<std::unique_ptr<Edge>> edges;

	void initiateEdges();
};


#endif