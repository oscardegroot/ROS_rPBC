/*
File: Agent.h

Define a connection with another agent that uses the ST and WVM to passify communication
*/

#ifndef Agent_H
#define Agent_H

#include "ros/ros.h"

#include "panda/Waves.h"
#include "std_msgs/Float64MultiArray.h"
#include "panda/getConnectionsOf.h"

#include "Agent.h"
#include "Edge.h"

#include <vector>
#include <string>
#include <sstream>

/* Possibly divide this up further in a system and a controller */

class Agent{

public:
	Agent();
	~Agent();

	virtual Eigen::VectorXd readSensors() = 0;
	virtual bool sendInput() = 0;
	virtual Eigen::MatrixXd getM(Eigen::VectorXd q) = 0;
	virtual Eigen::VectorXd getdHdq(Eigen::VectorXd q) = 0;

	Eigen::VectorXd sample();
	Eigen::VectorXd getOutput();

private:

	int i_id, j_id, l, N;

	// Coordinate count, actuated count
	const int nq = 0;
	const int mq = 0;

	ros::NodeHandlePtr n;

	ros::ServiceClient connect_client;
	std::vector<Edge*> edges;

	void initiateEdges();




};


#endif