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
#include "Goals.h"

#include <vector>
#include <string>
#include <sstream>

class Agent{

public:
	Agent();
	~Agent();


	void sample();

private:

	int i_id, j_id, l, N;


	ros::NodeHandlePtr n;

	ros::ServiceClient connect_client;
	std::vector<Edge*> edges;

	void initiateEdges();



};


#endif