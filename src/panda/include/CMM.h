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
#include "EdgeIntegral.h"
#include "EdgeFlex.h"
#include "EdgeDelayFree.h"
#include "EdgeFlexDelayFree.h"
#include "CustomLog.h"
#include "Helpers.h"

#include "panda/getConnectionsOf.h"

/* Possibly divide this up further in a system and a controller */

class CMM{

public:
	CMM(int set_id);
	~CMM();

	Eigen::VectorXd sample(Eigen::VectorXd r_i);
	void resetIntegrators();
	void activateIntegral();
	void deactivateIntegral();


	
private:

	int agent_id, l, N;
	Eigen::MatrixXd gain, integral_gain;
	bool integral_enabled;
	double torque_enable;

	std::vector<Eigen::VectorXd> integral_states;

	ros::NodeHandle n;
	ros::Time initial_time;

	ros::ServiceClient connect_client;
	std::vector<std::unique_ptr<Edge>> edges;
	//std::vector<Edge*> edges;
	std::vector<std::unique_ptr<Edge>> integral_edges;

	void initiateEdges();

};


#endif