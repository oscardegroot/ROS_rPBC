/*
File: Goals.h

Contains a number of functions describing formations and other cooperative goals
*/

#ifndef Goals_H
#define Goals_H

#include "ros/ros.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <string>
#include "panda/Waves.h"
#include "CustomLog.h"
#include "Exceptions.h"

#include "Helpers.h"

struct connection {
	int id;
	Eigen::VectorXd r_star;
};

struct leader {
    int id;
    Eigen::VectorXd ref;
    Eigen::VectorXd gain;
};

class Goals{

public:

	// The construct takes the number of agents
	Goals();
	~Goals();

	void Init(int N_set, int l_dim);

	// A function that sets a zero formation
	void setConsensus();
    void setCircle(const double radius, const double phase_offset);
    void setLine(const double spacing, const double angle);

	void setGraphFC();

	void initLeaders(ros::NodeHandle & nh);

	std::vector<connection> retrieveConnectionsOf(int agent_id);
	bool retrieveConnectionBetween(u_int id_i, u_int id_j, Eigen::VectorXd & r_star);
    bool isAgentLeader(u_int id, Eigen::VectorXd & ref, Eigen::VectorXd & gain);


private:
	
	// Agent count and cooperative dimension
	int N, l;

	// A vector with connections for every agent
	std::vector<std::vector<connection>> connections;

	// A bool matrix specifying connections
	std::vector<std::vector<bool>> graph;
	std::vector<leader> leaders;


};




#endif